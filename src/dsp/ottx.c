/*
 * OTTx Audio FX Plugin for Schwung
 *
 * Faithful scalar-C port of vitOTTx — Vital's 3-band multiband
 * upward+downward compressor.
 *
 * Copyright 2013-2019 Matt Tytel        (Vital DSP)
 *           2021      Yegor Suslin      (vitOTT)
 *           2026      Schwung OTTx port
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See the bundled LICENSE (GPLv3).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "host/plugin_api_v1.h"
#include "host/audio_fx_api_v2.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define OTTX_SR          44100.0f
#define OTTX_MAXBLK      128
#define OTTX_RMS_TIME    0.025f
#define OTTX_MAX_EXPAND  32.0f
#define OTTX_MIN_ENV     5.0f
#define OTTX_SQRT2       1.41421356237309504880f

/* Per-band base attack/release in ms (from compressor.cpp constants). */
#define OTTX_LOW_ATT  2.8f
#define OTTX_LOW_REL  40.0f
#define OTTX_MID_ATT  1.4f
#define OTTX_MID_REL  28.0f
#define OTTX_HIGH_ATT 0.7f
#define OTTX_HIGH_REL 15.0f

static const host_api_v1_t *g_host = NULL;

/* Flush-to-zero (FZ) on aarch64 — covers both denormal inputs and outputs
 * (AArch64 has no separate DAZ bit). Replaces JUCE ScopedNoDenormals.
 * No-op on non-aarch64 (native test host) so the harness still builds. */
static inline void ottx_set_flush_denormals(void) {
#if defined(__aarch64__)
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (1ULL << 24); /* FZ */
    __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#endif
}

/* ===== Vital fast-math (ported from vital_dsp/framework/futils.h) ===== */
/* kDbMagnitudeConversionMult = 1 / 6.02059991329 ; exp2(db*mult) == 10^(db/20) */
#define OTTX_DB_MAG_MULT   (1.0f / 6.02059991329f)
#define OTTX_EXP_CONV_MULT 1.44269504089f   /* 1/ln(2) */

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* futils::exp2 — quintic poly on fractional part, 2^integer via exponent bits */
static inline float vp_exp2(float exponent) {
    const float c0 = 1.0f;
    const float c1 = 16970.0f / 24483.0f;
    const float c2 = 1960.0f  / 8161.0f;
    const float c3 = 1360.0f  / 24483.0f;
    const float c4 = 80.0f    / 8161.0f;
    const float c5 = 32.0f    / 24483.0f;
    int integer = (int)lrintf(exponent);           /* roundToInt */
    float t = exponent - (float)integer;
    float int_pow;
    uint32_t bits = (uint32_t)(integer + 127) << 23; /* pow2ToFloat (unsigned shift: avoids signed-shift UB, bit-identical) */
    memcpy(&int_pow, &bits, sizeof(float));
    float cubic = t * (t * (t * c5 + c4) + c3) + c2;
    float interp = t * (t * cubic + c1) + c0;
    return int_pow * interp;
}

/* futils::log2 — quintic poly on mantissa, exponent from bits */
static inline float vp_log2(float value) {
    const float c0 = -1819.0f / 651.0f;
    const float c1 = 5.0f;
    const float c2 = -10.0f / 3.0f;
    const float c3 = 10.0f / 7.0f;
    const float c4 = -1.0f / 3.0f;
    const float c5 = 1.0f / 31.0f;
    uint32_t bits;
    memcpy(&bits, &value, sizeof(uint32_t));
    int floored = (int)(bits >> 23) - 0x7f;
    uint32_t t_bits = (bits & 0x7fffffu) | (0x7fu << 23);
    float t;
    memcpy(&t, &t_bits, sizeof(float));
    float cubic = t * (t * (t * c5 + c4) + c3) + c2;
    float interp = t * (t * cubic + c1) + c0;
    return (float)floored + interp;
}

static inline float vp_exp(float x)            { return vp_exp2(x * OTTX_EXP_CONV_MULT); }
static inline float vp_pow(float base, float e){ return vp_exp2(vp_log2(base) * e); }
static inline float vp_db_to_magnitude(float db){ return vp_exp2(db * OTTX_DB_MAG_MULT); }
static inline float vp_mul_add(float a, float b, float c) { return a + b * c; }
static inline float vp_interpolate(float from, float to, float t) { return from + t * (to - from); }

/* Minimal "key":number extractor for state restore (matches freeverb.c). */
static int ottx_json_get_float(const char *json, const char *key, float *out) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return -1;
    p += strlen(search);
    while (*p == ' ' || *p == '\t') p++;
    *out = (float)atof(p);
    return 0;
}

/* ---- one 2nd-order biquad section state ---- */
typedef struct { float in1, in2, out1, out2; } biquad_t;

/* ---- Linkwitz-Riley coefficient set (low + high share denominator) ---- */
typedef struct {
    float low_in0, low_in1, low_in2, low_out1, low_out2;
    float high_in0, high_in1, high_in2, high_out1, high_out2;
    float cutoff;
} lr_coeffs_t;

/* ---- one LR filter state: low path (a,b) + high path (a,b), per channel ---- */
typedef struct { biquad_t low_a, low_b, high_a, high_b; } lr_state_t;

/* ---- one compressor's per-channel running state ---- */
typedef struct {
    float low_env;   /* upward envelope (mean squared) */
    float high_env;  /* downward envelope (mean squared) */
    float out_mult;  /* ramped output-gain magnitude */
    float mix;       /* ramped wet/dry mix */
} comp_state_t;

/* ===== Linkwitz-Riley crossover (vital_dsp/linkwitz_riley_filter.cpp) ===== */
static void lr_coeffs(lr_coeffs_t *c, float cutoff, float sr) {
    cutoff = clampf(cutoff, 1.0f, 0.49f * sr);  /* guard tanf() against 0/Nyquist (div-by-zero) */
    c->cutoff = cutoff;
    float warp  = 1.0f / tanf((float)M_PI * cutoff / sr);
    float warp2 = warp * warp;
    float mult  = 1.0f / (1.0f + OTTX_SQRT2 * warp + warp2);
    c->low_in0  = mult;
    c->low_in1  = 2.0f * mult;
    c->low_in2  = mult;
    c->low_out1 = -2.0f * (1.0f - warp2) * mult;
    c->low_out2 = -(1.0f - OTTX_SQRT2 * warp + warp2) * mult;
    c->high_in0 = warp2 * mult;
    c->high_in1 = -2.0f * c->high_in0;
    c->high_in2 = c->high_in0;
    c->high_out1 = c->low_out1;
    c->high_out2 = c->low_out2;
}

/* One 2nd-order section. out1/out2 coeffs are the pre-negated denominator. */
static inline float biquad(biquad_t *s, float x,
                           float b0, float b1, float b2, float a1, float a2) {
    float y = vp_mul_add(vp_mul_add(vp_mul_add(vp_mul_add(x * b0, s->in1, b1),
                          s->in2, b2), s->out1, a1), s->out2, a2);
    s->in2 = s->in1; s->in1 = x;
    s->out2 = s->out1; s->out1 = y;
    return y;
}

/* 4th-order LR split: low = b(a(x)), high = b(a(x)) on high coeffs. */
static inline void lr_process(lr_state_t *s, const lr_coeffs_t *c, float x,
                              float *lo, float *hi) {
    float la = biquad(&s->low_a, x,  c->low_in0,  c->low_in1,  c->low_in2,  c->low_out1,  c->low_out2);
    *lo      = biquad(&s->low_b, la, c->low_in0,  c->low_in1,  c->low_in2,  c->low_out1,  c->low_out2);
    float ha = biquad(&s->high_a, x, c->high_in0, c->high_in1, c->high_in2, c->high_out1, c->high_out2);
    *hi      = biquad(&s->high_b, ha,c->high_in0, c->high_in1, c->high_in2, c->high_out1, c->high_out2);
}

/* ===== Compressor (vital_dsp/compressor.cpp Compressor::processRms / scaleOutput) =====
 * RMS-tracked dual envelope: upper (downward) + lower (upward), in power domain.
 * One comp_state_t per band per channel; thresholds/ratios shared across channels.
 */
static void comp_rms(comp_state_t *st, const float *in, float *out, int n,
                     float base_att_ms, float base_rel_ms,
                     float att_time, float rel_time,
                     float upper_thres_db, float lower_thres_db,
                     float upper_ratio, float lower_ratio, float sr) {
    float spm = sr / 1000.0f;
    float att_mult = base_att_ms * spm;
    float rel_mult = base_rel_ms * spm;
    float att_exp = clampf(att_time, 0.0f, 1.0f) * 8.0f - 4.0f;
    float rel_exp = clampf(rel_time, 0.0f, 1.0f) * 8.0f - 4.0f;
    float env_att = vp_exp(att_exp) * att_mult;
    float env_rel = vp_exp(rel_exp) * rel_mult;
    if (env_att < OTTX_MIN_ENV) env_att = OTTX_MIN_ENV;
    if (env_rel < OTTX_MIN_ENV) env_rel = OTTX_MIN_ENV;
    float att_scale = 1.0f / (env_att + 1.0f);
    float rel_scale = 1.0f / (env_rel + 1.0f);

    float ut = clampf(upper_thres_db, -100.0f, 12.0f);
    ut = vp_db_to_magnitude(ut); ut *= ut;                 /* power domain */
    float lt = clampf(lower_thres_db, -100.0f, 12.0f);
    lt = vp_db_to_magnitude(lt); lt *= lt;

    float ur = clampf(upper_ratio, 0.0f, 1.0f) * 0.5f;
    float lr = clampf(lower_ratio, -1.0f, 1.0f) * 0.5f;

    float henv = st->high_env, lenv = st->low_env;
    for (int i = 0; i < n; i++) {
        float s = in[i];
        float sq = s * s;

        /* downward: track upward-fast, floor at upper threshold */
        float hs, hsc;
        if (sq > henv) { hs = env_att; hsc = att_scale; } else { hs = env_rel; hsc = rel_scale; }
        henv = (sq + henv * hs) * hsc;
        if (henv < ut) henv = ut;
        float upper_mult = vp_pow(ut / henv, ur);

        /* upward: track, cap at lower threshold */
        float ls, lsc;
        if (sq > lenv) { ls = env_att; lsc = att_scale; } else { ls = env_rel; lsc = rel_scale; }
        lenv = (sq + lenv * ls) * lsc;
        if (lenv > lt) lenv = lt;
        float lower_mult = vp_pow(lt / lenv, lr);

        /* lenv is only capped (not floored), so on pure silence lenv can be 0 and
         * lt/lenv = +inf -> lower_mult is a huge finite value. The MAX_EXPAND clamp
         * below is the intended bound for this case (matches Vital); don't "fix" it. */
        float g = upper_mult * lower_mult;
        if (g < 0.0f) g = 0.0f;
        if (g > OTTX_MAX_EXPAND) g = OTTX_MAX_EXPAND;
        out[i] = g * s;
    }
    st->high_env = henv;
    st->low_env = lenv;
}

/* scaleOutput: ramp output gain + mix across the block, accumulate per band:
 *   acc[i] += interpolate(dry[i], comp[i]*out_mult, mix)
 * Summing all bands reconstructs the global dry/wet mix. */
static void comp_scale(comp_state_t *st, const float *dry, const float *comp,
                       float *acc, int n, float out_gain_db, float mix_target, float sr) {
    (void)sr;
    float cur_mult = st->out_mult;
    float tgt_mult = vp_db_to_magnitude(clampf(out_gain_db, -30.0f, 30.0f));
    st->out_mult = tgt_mult;
    float dmult = (tgt_mult - cur_mult) / (float)n;

    float cur_mix = st->mix;
    float tgt_mix = clampf(mix_target, 0.0f, 1.0f);
    st->mix = tgt_mix;
    float dmix = (tgt_mix - cur_mix) / (float)n;

    for (int i = 0; i < n; i++) {
        cur_mult += dmult;
        cur_mix += dmix;
        float wet = comp[i] * cur_mult;
        acc[i] += vp_interpolate(dry[i], wet, cur_mix);
    }
}

/* ---- instance ---- */
typedef struct {
    float sr;

    /* raw parameters (full 26, vitOTTx names) */
    float in_gain, out_gain, mix, depth, upward, downward, att_time, rel_time;
    float low_cross, high_cross;
    float lgain, mgain, hgain;
    float ll_thres, lu_thres, ll_ratio, lu_ratio;   /* low band */
    float bl_thres, bu_thres, bl_ratio, bu_ratio;   /* mid (band) */
    float hl_thres, hu_thres, hl_ratio, hu_ratio;   /* high band */
    int bypass;

    /* derived crossover coefficients */
    lr_coeffs_t c_low;   /* low/mid crossover */
    lr_coeffs_t c_high;  /* mid/high crossover */
    int mode;            /* 0 multiband, 3 singleband (see OTTX_MODE_*) */
    int prev_mode;

    /* per-channel filter + compressor state [0]=L [1]=R */
    lr_state_t f1[2];     /* stage 1 split at low_cross */
    lr_state_t f2lo[2];   /* stage 2 on low stream (phase-comp) */
    lr_state_t f3hi[2];   /* stage 2 on high stream (mid/high) */
    comp_state_t low_c[2], mid_c[2], high_c[2];

    /* scratch band buffers (per block, per channel processed serially) */
    float band_low[OTTX_MAXBLK], band_mid[OTTX_MAXBLK], band_high[OTTX_MAXBLK];
    float comp_scratch[OTTX_MAXBLK];
} ottx_t;

#define OTTX_MODE_MULTI  0
#define OTTX_MODE_SINGLE 3

static void v2_log(const char *msg) {
    if (g_host && g_host->log) {
        char b[256]; snprintf(b, sizeof(b), "[ottx] %s", msg); g_host->log(b);
    }
}

/* forward decls filled in later tasks */
static void ottx_update_derived(ottx_t *inst);
static void ottx_process_channel(ottx_t *inst, int ch, const float *in, float *out, int n);

static void ottx_set_defaults(ottx_t *inst) {
    inst->sr = OTTX_SR;
    inst->in_gain = 0.0f;  inst->out_gain = 0.0f;  inst->mix = 1.0f;
    inst->depth = 1.0f;    inst->upward = 1.0f;     inst->downward = 1.0f;
    inst->att_time = 0.5f; inst->rel_time = 0.5f;
    inst->low_cross = 120.0f; inst->high_cross = 2500.0f;
    inst->lgain = 16.3f; inst->mgain = 11.7f; inst->hgain = 16.3f;
    inst->ll_thres = -35.0f; inst->lu_thres = -28.0f; inst->ll_ratio = 0.8f; inst->lu_ratio = 0.9f;
    inst->bl_thres = -36.0f; inst->bu_thres = -25.0f; inst->bl_ratio = 0.8f; inst->bu_ratio = 0.857f;
    inst->hl_thres = -35.0f; inst->hu_thres = -30.0f; inst->hl_ratio = 0.8f; inst->hu_ratio = 1.0f;
    inst->bypass = 0;
    inst->mode = OTTX_MODE_MULTI;
    inst->prev_mode = OTTX_MODE_MULTI;
}

static void *v2_create_instance(const char *module_dir, const char *config_json) {
    (void)module_dir; (void)config_json;
    ottx_t *inst = (ottx_t *)calloc(1, sizeof(ottx_t));
    if (!inst) return NULL;
    ottx_set_defaults(inst);
    ottx_update_derived(inst);
    return inst;
}

static void v2_destroy_instance(void *instance) { free(instance); }

static void v2_process_block(void *instance, int16_t *audio_inout, int frames) {
    ottx_t *inst = (ottx_t *)instance;
    if (!inst || frames <= 0) return;

    if (inst->bypass) return;   /* pass through untouched */

    ottx_set_flush_denormals();

    float in_mag  = vp_db_to_magnitude(clampf(inst->in_gain,  -60.0f, 30.0f));
    float out_mag = vp_db_to_magnitude(clampf(inst->out_gain, -60.0f, 30.0f));

    for (int off = 0; off < frames; ) {
        int n = frames - off;
        if (n > OTTX_MAXBLK) n = OTTX_MAXBLK;

        float in_l[OTTX_MAXBLK], in_r[OTTX_MAXBLK];
        float out_l[OTTX_MAXBLK], out_r[OTTX_MAXBLK];
        for (int i = 0; i < n; i++) {
            in_l[i] = (audio_inout[(off + i) * 2]     / 32768.0f) * in_mag;
            in_r[i] = (audio_inout[(off + i) * 2 + 1] / 32768.0f) * in_mag;
        }

        ottx_process_channel(inst, 0, in_l, out_l, n);
        ottx_process_channel(inst, 1, in_r, out_r, n);

        for (int i = 0; i < n; i++) {
            float l = out_l[i] * out_mag;
            float r = out_r[i] * out_mag;
            l = clampf(l, -1.0f, 1.0f);
            r = clampf(r, -1.0f, 1.0f);
            audio_inout[(off + i) * 2]     = (int16_t)lrintf(l * 32767.0f);
            audio_inout[(off + i) * 2 + 1] = (int16_t)lrintf(r * 32767.0f);
        }
        off += n;
    }
}

static void v2_set_param(void *instance, const char *key, const char *val) {
    ottx_t *p = (ottx_t *)instance;
    if (!p || !key || !val) return;
    if (strcmp(key, "state") == 0) {
        float v;
        #define OTTX_RESTORE(k, field) if (ottx_json_get_float(val, k, &v) == 0) p->field = v
        OTTX_RESTORE("in_gain", in_gain);   OTTX_RESTORE("out_gain", out_gain);
        OTTX_RESTORE("mix", mix);           OTTX_RESTORE("depth", depth);
        OTTX_RESTORE("upward", upward);     OTTX_RESTORE("downward", downward);
        OTTX_RESTORE("att_time", att_time); OTTX_RESTORE("rel_time", rel_time);
        OTTX_RESTORE("low_cross", low_cross); OTTX_RESTORE("high_cross", high_cross);
        OTTX_RESTORE("lgain", lgain);       OTTX_RESTORE("mgain", mgain); OTTX_RESTORE("hgain", hgain);
        OTTX_RESTORE("ll_thres", ll_thres); OTTX_RESTORE("lu_thres", lu_thres);
        OTTX_RESTORE("ll_ratio", ll_ratio); OTTX_RESTORE("lu_ratio", lu_ratio);
        OTTX_RESTORE("bl_thres", bl_thres); OTTX_RESTORE("bu_thres", bu_thres);
        OTTX_RESTORE("bl_ratio", bl_ratio); OTTX_RESTORE("bu_ratio", bu_ratio);
        OTTX_RESTORE("hl_thres", hl_thres); OTTX_RESTORE("hu_thres", hu_thres);
        OTTX_RESTORE("hl_ratio", hl_ratio); OTTX_RESTORE("hu_ratio", hu_ratio);
        #undef OTTX_RESTORE
        ottx_update_derived(p);
        return;
    }
    float v = (float)atof(val);
    if      (strcmp(key, "in_gain") == 0)    p->in_gain = v;
    else if (strcmp(key, "out_gain") == 0)   p->out_gain = v;
    else if (strcmp(key, "mix") == 0)        p->mix = v;
    else if (strcmp(key, "depth") == 0)      p->depth = v;
    else if (strcmp(key, "upward") == 0)     p->upward = v;
    else if (strcmp(key, "downward") == 0)   p->downward = v;
    else if (strcmp(key, "att_time") == 0)   p->att_time = v;
    else if (strcmp(key, "rel_time") == 0)   p->rel_time = v;
    else if (strcmp(key, "time") == 0)     { p->att_time = v; p->rel_time = v; }
    else if (strcmp(key, "low_cross") == 0)  p->low_cross = v;
    else if (strcmp(key, "high_cross") == 0) p->high_cross = v;
    else if (strcmp(key, "lgain") == 0)      p->lgain = v;
    else if (strcmp(key, "mgain") == 0)      p->mgain = v;
    else if (strcmp(key, "hgain") == 0)      p->hgain = v;
    else if (strcmp(key, "ll_thres") == 0)   p->ll_thres = v;
    else if (strcmp(key, "lu_thres") == 0)   p->lu_thres = v;
    else if (strcmp(key, "ll_ratio") == 0)   p->ll_ratio = v;
    else if (strcmp(key, "lu_ratio") == 0)   p->lu_ratio = v;
    else if (strcmp(key, "bl_thres") == 0)   p->bl_thres = v;
    else if (strcmp(key, "bu_thres") == 0)   p->bu_thres = v;
    else if (strcmp(key, "bl_ratio") == 0)   p->bl_ratio = v;
    else if (strcmp(key, "bu_ratio") == 0)   p->bu_ratio = v;
    else if (strcmp(key, "hl_thres") == 0)   p->hl_thres = v;
    else if (strcmp(key, "hu_thres") == 0)   p->hu_thres = v;
    else if (strcmp(key, "hl_ratio") == 0)   p->hl_ratio = v;
    else if (strcmp(key, "hu_ratio") == 0)   p->hu_ratio = v;
    else if (strcmp(key, "bypass") == 0)     p->bypass = (v >= 0.5f) ? 1 : 0;
    else return;
    ottx_update_derived(p);
}

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len) {
    ottx_t *p = (ottx_t *)instance;
    if (!p) return -1;
    #define OTTX_GET(k, field) if (strcmp(key, k) == 0) return snprintf(buf, buf_len, "%.4f", p->field)
    OTTX_GET("in_gain", in_gain);   OTTX_GET("out_gain", out_gain);
    OTTX_GET("mix", mix);           OTTX_GET("depth", depth);
    OTTX_GET("upward", upward);     OTTX_GET("downward", downward);
    OTTX_GET("att_time", att_time); OTTX_GET("rel_time", rel_time);
    OTTX_GET("low_cross", low_cross); OTTX_GET("high_cross", high_cross);
    OTTX_GET("lgain", lgain);       OTTX_GET("mgain", mgain); OTTX_GET("hgain", hgain);
    OTTX_GET("ll_thres", ll_thres); OTTX_GET("lu_thres", lu_thres);
    OTTX_GET("ll_ratio", ll_ratio); OTTX_GET("lu_ratio", lu_ratio);
    OTTX_GET("bl_thres", bl_thres); OTTX_GET("bu_thres", bu_thres);
    OTTX_GET("bl_ratio", bl_ratio); OTTX_GET("bu_ratio", bu_ratio);
    OTTX_GET("hl_thres", hl_thres); OTTX_GET("hu_thres", hu_thres);
    OTTX_GET("hl_ratio", hl_ratio); OTTX_GET("hu_ratio", hu_ratio);
    #undef OTTX_GET

    /* "time" is a write-macro that sets att_time + rel_time together. The shadow
     * UI reads a param's current value via get_param to make it editable, so it
     * MUST be gettable or the editor shows "nothing to adjust". Report att_time
     * as the macro's representative value. */
    if (strcmp(key, "time") == 0) return snprintf(buf, buf_len, "%.4f", p->att_time);

    if (strcmp(key, "name") == 0) return snprintf(buf, buf_len, "OTTx");

    /* chain_params: per-param metadata (name/range/unit/step) served live from
     * the plugin. The Master/Send FX bus reads this from the .so's get_param
     * (shadow_chain_mgmt.c); without it the bus falls back to a stale cached
     * module.json parse, so labels/ranges never refresh. Keep in sync with
     * module.json's ui_hierarchy param objects (single source of truth). */
    if (strcmp(key, "chain_params") == 0) {
        typedef struct { const char *key, *name, *unit; float min, max, def, step; } ottx_pmeta_t;
        static const ottx_pmeta_t OTTX_PMETA[] = {
            {"mix","Mix","%",0.0f,1.0f,1.0f,0.02f},
            {"depth","Depth","%",0.0f,1.0f,1.0f,0.02f},
            {"upward","Upward","",0.0f,2.0f,1.0f,0.02f},
            {"downward","Downward","",0.0f,2.0f,1.0f,0.02f},
            {"time","Time","%",0.0f,1.0f,0.5f,0.02f},
            {"in_gain","In Gain","dB",-60.0f,30.0f,0.0f,0.5f},
            {"out_gain","Out Gain","dB",-60.0f,30.0f,0.0f,0.5f},
            {"low_cross","Low/Mid Hz","Hz",20.0f,18000.0f,120.0f,10.0f},
            {"high_cross","Mid/Hi Hz","Hz",20.0f,18000.0f,2500.0f,10.0f},
            {"att_time","Attack","%",0.0f,1.0f,0.5f,0.02f},
            {"rel_time","Release","%",0.0f,1.0f,0.5f,0.02f},
            {"ll_thres","Low Up Thr","dB",-80.0f,0.0f,-35.0f,1.0f},
            {"ll_ratio","Low Up Ratio","%",-1.0f,1.0f,0.8f,0.02f},
            {"lu_thres","Low Dn Thr","dB",-80.0f,0.0f,-28.0f,1.0f},
            {"lu_ratio","Low Dn Ratio","%",0.0f,1.0f,0.9f,0.02f},
            {"lgain","Low Gain","dB",-30.0f,30.0f,16.3f,0.5f},
            {"bl_thres","Mid Up Thr","dB",-80.0f,0.0f,-36.0f,1.0f},
            {"bl_ratio","Mid Up Ratio","%",-1.0f,1.0f,0.8f,0.02f},
            {"bu_thres","Mid Dn Thr","dB",-80.0f,0.0f,-25.0f,1.0f},
            {"bu_ratio","Mid Dn Ratio","%",0.0f,1.0f,0.857f,0.02f},
            {"mgain","Mid Gain","dB",-30.0f,30.0f,11.7f,0.5f},
            {"hl_thres","Hi Up Thr","dB",-80.0f,0.0f,-35.0f,1.0f},
            {"hl_ratio","Hi Up Ratio","%",-1.0f,1.0f,0.8f,0.02f},
            {"hu_thres","Hi Dn Thr","dB",-80.0f,0.0f,-30.0f,1.0f},
            {"hu_ratio","Hi Dn Ratio","%",0.0f,1.0f,1.0f,0.02f},
            {"hgain","Hi Gain","dB",-30.0f,30.0f,16.3f,0.5f},
        };
        int count = (int)(sizeof(OTTX_PMETA) / sizeof(OTTX_PMETA[0]));
        int n = 0;
        n += snprintf(buf + n, buf_len - n, "[");
        for (int i = 0; i < count && n < buf_len; i++) {
            const ottx_pmeta_t *m = &OTTX_PMETA[i];
            n += snprintf(buf + n, buf_len - n,
                "%s{\"key\":\"%s\",\"name\":\"%s\",\"type\":\"float\","
                "\"min\":%g,\"max\":%g,\"default\":%g,\"step\":%g%s%s%s}",
                i ? "," : "", m->key, m->name,
                (double)m->min, (double)m->max, (double)m->def, (double)m->step,
                m->unit[0] ? ",\"unit\":\"" : "", m->unit, m->unit[0] ? "\"" : "");
        }
        n += snprintf(buf + n, buf_len - n, "]");
        return n;
    }

    if (strcmp(key, "state") == 0) {
        return snprintf(buf, buf_len,
            "{\"in_gain\":%.4f,\"out_gain\":%.4f,\"mix\":%.4f,\"depth\":%.4f,"
            "\"upward\":%.4f,\"downward\":%.4f,\"att_time\":%.4f,\"rel_time\":%.4f,"
            "\"low_cross\":%.4f,\"high_cross\":%.4f,\"lgain\":%.4f,\"mgain\":%.4f,\"hgain\":%.4f,"
            "\"ll_thres\":%.4f,\"lu_thres\":%.4f,\"ll_ratio\":%.4f,\"lu_ratio\":%.4f,"
            "\"bl_thres\":%.4f,\"bu_thres\":%.4f,\"bl_ratio\":%.4f,\"bu_ratio\":%.4f,"
            "\"hl_thres\":%.4f,\"hu_thres\":%.4f,\"hl_ratio\":%.4f,\"hu_ratio\":%.4f}",
            p->in_gain, p->out_gain, p->mix, p->depth, p->upward, p->downward,
            p->att_time, p->rel_time, p->low_cross, p->high_cross,
            p->lgain, p->mgain, p->hgain,
            p->ll_thres, p->lu_thres, p->ll_ratio, p->lu_ratio,
            p->bl_thres, p->bu_thres, p->bl_ratio, p->bu_ratio,
            p->hl_thres, p->hu_thres, p->hl_ratio, p->hu_ratio);
    }

    if (strcmp(key, "ui_hierarchy") == 0) {
        const char *h =
            "{\"modes\":null,\"levels\":{"
              "\"root\":{"
                "\"children\":null,"
                "\"knobs\":[\"mix\",\"depth\",\"upward\",\"downward\",\"time\",\"in_gain\",\"out_gain\"],"
                /* The Advanced submenu link MUST be a {"level":...} entry inside
                 * params — the shadow-UI renderer only navigates params entries,
                 * never a bare children[] array. */
                "\"params\":[\"mix\",\"depth\",\"upward\",\"downward\",\"time\",\"in_gain\",\"out_gain\",\"low_cross\",\"high_cross\",{\"level\":\"advanced\",\"label\":\"Advanced\"}]"
              "},"
              "\"advanced\":{"
                "\"children\":null,"
                /* Crossovers live on the root list only (shared between bands);
                 * don't duplicate them here. Advanced is the per-band detail,
                 * flat and ordered Low -> Mid -> High. */
                "\"knobs\":[\"att_time\",\"rel_time\",\"lgain\",\"mgain\",\"hgain\"],"
                /* Per band, pair each threshold with its ratio: Up Thr, Up Ratio,
                 * Dn Thr, Dn Ratio, Gain. */
                "\"params\":[\"att_time\",\"rel_time\","
                  "\"ll_thres\",\"ll_ratio\",\"lu_thres\",\"lu_ratio\",\"lgain\","
                  "\"bl_thres\",\"bl_ratio\",\"bu_thres\",\"bu_ratio\",\"mgain\","
                  "\"hl_thres\",\"hl_ratio\",\"hu_thres\",\"hu_ratio\",\"hgain\"]"
              "}"
            "}}";
        int len = (int)strlen(h);
        if (len < buf_len) { strcpy(buf, h); return len; }
        return -1;
    }
    return -1;
}

/* Effective per-band ratios after the OTT macro mapping (depth/upward/downward). */
typedef struct {
    float ll, lu, bl, bu, hl, hu;   /* low/mid/high lower(upward) + upper(downward) */
} ottx_eff_ratios_t;

static ottx_eff_ratios_t ottx_eff_ratios(const ottx_t *p) {
    ottx_eff_ratios_t r;
    float d = p->depth, up = p->upward, dn = p->downward;
    r.ll = p->ll_ratio * d * up;  r.lu = p->lu_ratio * d * dn;
    r.bl = p->bl_ratio * d * up;  r.bu = p->bu_ratio * d * dn;
    r.hl = p->hl_ratio * d * up;  r.hu = p->hu_ratio * d * dn;
    return r;
}

/* Recompute crossover coefficients + collapse mode from current params. */
static void ottx_update_derived(ottx_t *inst) {
    float lf = inst->low_cross, hf = inst->high_cross;
    if (lf > hf) lf = hf;
    lr_coeffs(&inst->c_low,  lf, inst->sr);
    lr_coeffs(&inst->c_high, hf, inst->sr);

    int low_collapsed  = (lf <= 21.0f);
    int high_collapsed = (hf >= 17500.0f);
    /* v1: full multiband unless BOTH crossovers collapse -> singleband (mid settings). */
    inst->mode = (low_collapsed && high_collapsed) ? OTTX_MODE_SINGLE : OTTX_MODE_MULTI;
}

/* Process one channel's block (float in [-1,1]) -> float out. */
static void ottx_process_channel(ottx_t *inst, int ch, const float *in, float *out, int n) {
    if (n > OTTX_MAXBLK) n = OTTX_MAXBLK;  /* scratch buffers are OTTX_MAXBLK; callers chunk, but guard anyway */
    if (inst->mode != inst->prev_mode) {
        memset(inst->f1, 0, sizeof(inst->f1));
        memset(inst->f2lo, 0, sizeof(inst->f2lo));
        memset(inst->f3hi, 0, sizeof(inst->f3hi));
        memset(inst->low_c, 0, sizeof(inst->low_c));
        memset(inst->mid_c, 0, sizeof(inst->mid_c));
        memset(inst->high_c, 0, sizeof(inst->high_c));
        inst->prev_mode = inst->mode;
    }

    ottx_eff_ratios_t er = ottx_eff_ratios(inst);

    if (inst->mode == OTTX_MODE_SINGLE) {
        /* one compressor (mid settings) on the full signal */
        comp_rms(&inst->mid_c[ch], in, inst->comp_scratch, n,
                 OTTX_MID_ATT, OTTX_MID_REL, inst->att_time, inst->rel_time,
                 inst->bu_thres, inst->bl_thres, er.bu, er.bl, inst->sr);
        for (int i = 0; i < n; i++) out[i] = 0.0f;
        comp_scale(&inst->mid_c[ch], in, inst->comp_scratch, out, n, inst->mgain, inst->mix, inst->sr);
        return;
    }

    /* --- split into 3 bands (faithful to MultibandCompressor lane logic) --- */
    for (int i = 0; i < n; i++) {
        float x = in[i];
        float lowA, highA;
        lr_process(&inst->f1[ch], &inst->c_low, x, &lowA, &highA);

        float llp, lhp;
        lr_process(&inst->f2lo[ch], &inst->c_high, lowA, &llp, &lhp);
        inst->band_low[i] = llp + lhp;     /* phase-comp reconstruction of LOW */

        float mlp, mhp;
        lr_process(&inst->f3hi[ch], &inst->c_high, highA, &mlp, &mhp);
        inst->band_mid[i] = mlp;
        inst->band_high[i] = mhp;
    }

    for (int i = 0; i < n; i++) out[i] = 0.0f;

    /* LOW band */
    comp_rms(&inst->low_c[ch], inst->band_low, inst->comp_scratch, n,
             OTTX_LOW_ATT, OTTX_LOW_REL, inst->att_time, inst->rel_time,
             inst->lu_thres, inst->ll_thres, er.lu, er.ll, inst->sr);
    comp_scale(&inst->low_c[ch], inst->band_low, inst->comp_scratch, out, n,
               inst->lgain, inst->mix, inst->sr);

    /* MID band */
    comp_rms(&inst->mid_c[ch], inst->band_mid, inst->comp_scratch, n,
             OTTX_MID_ATT, OTTX_MID_REL, inst->att_time, inst->rel_time,
             inst->bu_thres, inst->bl_thres, er.bu, er.bl, inst->sr);
    comp_scale(&inst->mid_c[ch], inst->band_mid, inst->comp_scratch, out, n,
               inst->mgain, inst->mix, inst->sr);

    /* HIGH band */
    comp_rms(&inst->high_c[ch], inst->band_high, inst->comp_scratch, n,
             OTTX_HIGH_ATT, OTTX_HIGH_REL, inst->att_time, inst->rel_time,
             inst->hu_thres, inst->hl_thres, er.hu, er.hl, inst->sr);
    comp_scale(&inst->high_c[ch], inst->band_high, inst->comp_scratch, out, n,
               inst->hgain, inst->mix, inst->sr);
}

#ifndef OTTX_TEST
static audio_fx_api_v2_t g_fx_api_v2;
audio_fx_api_v2_t *move_audio_fx_init_v2(const host_api_v1_t *host) {
    g_host = host;
    memset(&g_fx_api_v2, 0, sizeof(g_fx_api_v2));
    g_fx_api_v2.api_version = AUDIO_FX_API_VERSION_2;
    g_fx_api_v2.create_instance = v2_create_instance;
    g_fx_api_v2.destroy_instance = v2_destroy_instance;
    g_fx_api_v2.process_block = v2_process_block;
    g_fx_api_v2.set_param = v2_set_param;
    g_fx_api_v2.get_param = v2_get_param;
    v2_log("OTTx v2 plugin initialized");
    return &g_fx_api_v2;
}
#endif
