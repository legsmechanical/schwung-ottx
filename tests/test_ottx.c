#define OTTX_TEST
#include "ottx.c"

#include <stdio.h>
#include <math.h>

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
    else { printf("ok: %s\n", msg); } } while (0)
#define CHECK_NEAR(a, b, eps, msg) CHECK(fabsf((float)(a) - (float)(b)) <= (eps), msg)

static void test_fastmath(void) {
    /* dbToMagnitude: 0 dB -> 1.0, -6.0206 dB -> 0.5, +6.0206 dB -> 2.0 */
    CHECK_NEAR(vp_db_to_magnitude(0.0f),   1.0f, 0.01f, "db_to_mag 0dB == 1");
    CHECK_NEAR(vp_db_to_magnitude(-6.0206f), 0.5f, 0.01f, "db_to_mag -6dB == 0.5");
    CHECK_NEAR(vp_db_to_magnitude(6.0206f), 2.0f, 0.02f, "db_to_mag +6dB == 2.0");
    /* exp/pow sanity */
    CHECK_NEAR(vp_exp(0.0f), 1.0f, 0.01f, "exp(0)==1");
    CHECK_NEAR(vp_exp(1.0f), 2.71828f, 0.02f, "exp(1)==e");
    CHECK_NEAR(vp_pow(2.0f, 10.0f), 1024.0f, 4.0f, "pow(2,10)==1024");
    CHECK_NEAR(vp_pow(9.0f, 0.5f), 3.0f, 0.03f, "pow(9,0.5)==3");
    CHECK_NEAR(vp_mul_add(1.0f, 2.0f, 3.0f), 7.0f, 0.0001f, "mul_add(1,2,3)==7");
    CHECK_NEAR(vp_interpolate(10.0f, 20.0f, 0.25f), 12.5f, 0.0001f, "interp 0.25");
}

static void test_lr_filter(void) {
    lr_coeffs_t c;
    lr_coeffs(&c, 1000.0f, OTTX_SR);

    /* A 4th-order Linkwitz-Riley crossover sums to an ALLPASS: low+high has flat
     * magnitude vs input but is phase-rotated, so it matches in RMS/energy, NOT
     * sample-for-sample. Assert magnitude (energy) preservation, not identity. */
    lr_state_t s; memset(&s, 0, sizeof(s));
    float in_e = 0.0f, sum_e = 0.0f; int n = 4000;
    for (int i = 0; i < n; i++) {
        float x = sinf(2.0f * (float)M_PI * 200.0f * i / OTTX_SR);
        float lo, hi; lr_process(&s, &c, x, &lo, &hi);
        if (i > 2000) { in_e += x * x; sum_e += (lo + hi) * (lo + hi); } /* after settling */
    }
    CHECK(fabsf(sqrtf(sum_e / in_e) - 1.0f) < 0.05f, "LR low+high preserves magnitude (allpass sum)");

    /* A 50 Hz tone is mostly in the LOW output for a 1 kHz crossover. */
    lr_state_t s2; memset(&s2, 0, sizeof(s2));
    float low_e = 0, high_e = 0;
    for (int i = 0; i < 8000; i++) {
        float x = sinf(2.0f * (float)M_PI * 50.0f * i / OTTX_SR);
        float lo, hi; lr_process(&s2, &c, x, &lo, &hi);
        if (i > 4000) { low_e += lo*lo; high_e += hi*hi; }
    }
    CHECK(low_e > high_e * 20.0f, "50Hz tone dominated by LOW band at 1kHz crossover");

    /* A 6 kHz tone is mostly in the HIGH output. */
    lr_state_t s3; memset(&s3, 0, sizeof(s3));
    low_e = 0; high_e = 0;
    for (int i = 0; i < 8000; i++) {
        float x = sinf(2.0f * (float)M_PI * 6000.0f * i / OTTX_SR);
        float lo, hi; lr_process(&s3, &c, x, &lo, &hi);
        if (i > 4000) { low_e += lo*lo; high_e += hi*hi; }
    }
    CHECK(high_e > low_e * 20.0f, "6kHz tone dominated by HIGH band at 1kHz crossover");
}

static void test_passthrough_when_dry(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
    CHECK(inst != NULL, "instance created");
    v2_set_param(inst, "mix", "0");        /* fully dry */
    v2_set_param(inst, "in_gain", "0");
    v2_set_param(inst, "out_gain", "0");

    /* At mix=0 the dry tap is the per-band crossover reconstruction (a cascade of
     * allpasses) — this matches vitOTTx, which mixes against the POST-split band
     * signal, not the original input. So the dry output preserves ENERGY but is
     * phase-rotated, not sample-identical. Assert energy preservation; bit-exact
     * passthrough is what the bypass control provides (see test_bypass). */
    double in_e = 0.0, out_e = 0.0;
    int phase = 0;
    for (int blk = 0; blk < 40; blk++) {
        int16_t buf[256];
        for (int i = 0; i < 128; i++) {
            int16_t s = (int16_t)(10000.0 * sin((phase + i) * 0.13));
            buf[i * 2] = s; buf[i * 2 + 1] = s;
        }
        phase += 128;
        int16_t in_copy[256]; memcpy(in_copy, buf, sizeof(buf));
        v2_process_block(inst, buf, 128);  /* 128 stereo frames */
        if (blk >= 30) for (int i = 0; i < 256; i++) {
            in_e  += (double)in_copy[i] * in_copy[i];
            out_e += (double)buf[i]     * buf[i];
        }
    }
    CHECK(fabs(sqrt(out_e / in_e) - 1.0) < 0.05, "mix=0 dry output preserves signal energy (allpass)");
    v2_destroy_instance(inst);
}

static void test_bypass(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
    v2_set_param(inst, "bypass", "1");
    int16_t buf[16];
    for (int i = 0; i < 16; i++) buf[i] = (int16_t)(i * 1500 - 12000);
    int16_t expect[16]; memcpy(expect, buf, sizeof(buf));
    v2_process_block(inst, buf, 8);
    int ok = 1; for (int i = 0; i < 16; i++) if (buf[i] != expect[i]) ok = 0;
    CHECK(ok, "bypass passes audio through unchanged");
    v2_destroy_instance(inst);
}

static void test_compressor(void) {
    /* Silence -> silence (and envelopes stay finite). */
    comp_state_t st; memset(&st, 0, sizeof(st));
    float in[OTTX_MAXBLK] = {0}, out[OTTX_MAXBLK] = {0};
    comp_rms(&st, in, out, 64, OTTX_MID_ATT, OTTX_MID_REL,
             0.5f, 0.5f, -25.0f, -36.0f, 0.857f, 0.8f, OTTX_SR);
    int silent = 1;
    for (int i = 0; i < 64; i++) if (fabsf(out[i]) > 1e-6f) silent = 0;
    CHECK(silent, "silence in -> silence out");

    /* Quiet tone well below thresholds gets UPWARD-boosted (gain > 1). */
    comp_state_t su; memset(&su, 0, sizeof(su));
    float quiet_in_rms = 0, quiet_out_rms = 0;
    for (int blk = 0; blk < 200; blk++) {
        for (int i = 0; i < 64; i++)
            in[i] = 0.01f * sinf(2.0f*(float)M_PI*300.0f*(blk*64+i)/OTTX_SR); /* ~-40 dBFS */
        comp_rms(&su, in, out, 64, OTTX_MID_ATT, OTTX_MID_REL,
                 0.5f, 0.5f, -25.0f, -36.0f, 0.857f, 0.8f, OTTX_SR);
        if (blk >= 150) for (int i = 0; i < 64; i++) { quiet_in_rms += in[i]*in[i]; quiet_out_rms += out[i]*out[i]; }
    }
    CHECK(quiet_out_rms > quiet_in_rms * 1.5f, "quiet signal is boosted (upward compression)");

    /* Loud tone above upper threshold gets DOWNWARD-compressed (gain < 1). */
    comp_state_t sd; memset(&sd, 0, sizeof(sd));
    float loud_in_rms = 0, loud_out_rms = 0;
    for (int blk = 0; blk < 200; blk++) {
        for (int i = 0; i < 64; i++)
            in[i] = 0.7f * sinf(2.0f*(float)M_PI*300.0f*(blk*64+i)/OTTX_SR); /* ~-3 dBFS */
        comp_rms(&sd, in, out, 64, OTTX_MID_ATT, OTTX_MID_REL,
                 0.5f, 0.5f, -25.0f, -36.0f, 0.857f, 0.8f, OTTX_SR);
        if (blk >= 150) for (int i = 0; i < 64; i++) { loud_in_rms += in[i]*in[i]; loud_out_rms += out[i]*out[i]; }
    }
    CHECK(loud_out_rms < loud_in_rms, "loud signal is attenuated (downward compression)");

    /* scaleOutput with mix=0 -> output equals dry (accumulator). */
    comp_state_t ss; memset(&ss, 0, sizeof(ss));
    float dry[8], comp[8], acc[8];
    for (int i = 0; i < 8; i++) { dry[i] = 0.3f; comp[i] = 0.9f; acc[i] = 0.0f; }
    /* prime ramps so mix is exactly 0 across the block */
    ss.out_mult = vp_db_to_magnitude(0.0f); ss.mix = 0.0f;
    comp_scale(&ss, dry, comp, acc, 8, /*out_gain_db*/0.0f, /*mix*/0.0f, OTTX_SR);
    int dryok = 1; for (int i = 0; i < 8; i++) if (fabsf(acc[i] - dry[i]) > 1e-4f) dryok = 0;
    CHECK(dryok, "comp_scale mix=0 accumulates dry signal");
}

static void test_multiband(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
    /* Unity-ish settings: depth small so it barely compresses; mix=1; band gains 0 dB. */
    v2_set_param(inst, "depth", "0");      /* ratios -> 0 -> gain ~1 everywhere */
    v2_set_param(inst, "lgain", "0");
    v2_set_param(inst, "mgain", "0");
    v2_set_param(inst, "hgain", "0");
    v2_set_param(inst, "mix", "1");

    /* Feed a broadband-ish signal; output should track input magnitude (reconstruction). */
    float in_rms = 0, out_rms = 0, diff = 0;
    for (int blk = 0; blk < 300; blk++) {
        float buf_in[OTTX_MAXBLK];
        for (int i = 0; i < 64; i++) {
            float t = (blk*64+i) / OTTX_SR;
            buf_in[i] = 0.2f*(sinf(2*(float)M_PI*100*t) + sinf(2*(float)M_PI*1500*t) + sinf(2*(float)M_PI*7000*t))/3.0f;
        }
        float buf_out[OTTX_MAXBLK];
        ottx_process_channel(inst, 0, buf_in, buf_out, 64);
        if (blk >= 250) for (int i = 0; i < 64; i++) {
            in_rms += buf_in[i]*buf_in[i]; out_rms += buf_out[i]*buf_out[i];
            diff += fabsf(buf_out[i] - buf_in[i]);
        }
    }
    CHECK(out_rms > in_rms * 0.5f && out_rms < in_rms * 2.0f,
          "depth=0 reconstruction keeps level roughly unity");
    v2_destroy_instance(inst);

    /* No NaN/Inf with extreme crossovers (collapse path). */
    ottx_t *inst2 = (ottx_t *)v2_create_instance(NULL, NULL);
    v2_set_param(inst2, "low_cross", "20");
    v2_set_param(inst2, "high_cross", "18000");
    float bi[OTTX_MAXBLK], bo[OTTX_MAXBLK];
    int finite = 1;
    for (int blk = 0; blk < 50; blk++) {
        for (int i = 0; i < 64; i++) bi[i] = 0.3f*sinf(2*(float)M_PI*440*(blk*64+i)/OTTX_SR);
        ottx_process_channel(inst2, 0, bi, bo, 64);
        for (int i = 0; i < 64; i++) if (!isfinite(bo[i])) finite = 0;
    }
    CHECK(finite, "collapse-mode crossovers produce finite output");
    v2_destroy_instance(inst2);
}

static void test_params(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
    char buf[2048];

    v2_set_param(inst, "depth", "0.5");
    int len = v2_get_param(inst, "depth", buf, sizeof(buf));
    CHECK(len > 0 && fabsf((float)atof(buf) - 0.5f) < 1e-3f, "get_param depth round-trips");

    /* state save -> mutate -> restore */
    v2_set_param(inst, "lgain", "5.5");
    int slen = v2_get_param(inst, "state", buf, sizeof(buf));
    CHECK(slen > 0 && strstr(buf, "lgain") != NULL, "state JSON contains params");
    char saved[2048]; memcpy(saved, buf, slen + 1);
    v2_set_param(inst, "lgain", "-12.0");
    v2_set_param(inst, "state", saved);
    v2_get_param(inst, "lgain", buf, sizeof(buf));
    CHECK(fabsf((float)atof(buf) - 5.5f) < 1e-2f, "state restore recovers lgain");

    int hlen = v2_get_param(inst, "ui_hierarchy", buf, sizeof(buf));
    CHECK(hlen > 0 && strstr(buf, "depth") != NULL, "ui_hierarchy echoes");

    int nlen = v2_get_param(inst, "name", buf, sizeof(buf));
    CHECK(nlen > 0 && strcmp(buf, "OTTx") == 0, "name == OTTx");

    v2_destroy_instance(inst);
}

static void test_ui_hierarchy_submenu(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
    char buf[2048];
    int len = v2_get_param(inst, "ui_hierarchy", buf, sizeof(buf));
    CHECK(len > 0, "ui_hierarchy returned");

    /* The shadow-UI renderer (loadHierarchyLevel in shadow_ui.js) builds the
     * navigable list ONLY from a level's "params" array; a submenu must be a
     * {"level":...} entry INSIDE params. A bare "children":[...] array is never
     * rendered as a list item. So the Advanced link must sit in root.params,
     * right after the last root param (high_cross). */
    CHECK(strstr(buf, "\"high_cross\",{\"level\":\"advanced\",\"label\":\"Advanced\"}]") != NULL,
          "root params includes the Advanced submenu nav-link (renderer contract)");
    /* And the 'advanced' level must exist as a target. */
    CHECK(strstr(buf, "\"advanced\":{") != NULL, "advanced level defined");

    /* Crossovers live on root only — the advanced params must NOT re-list them
     * (they showed in both menus otherwise). Advanced now starts with att_time. */
    CHECK(strstr(buf, "\"params\":[\"att_time\",\"rel_time\",\"ll_thres\"") != NULL,
          "advanced params start at att_time (crossovers not duplicated)");

    /* "time" must be gettable, or the shadow UI shows 'nothing to adjust'. */
    v2_set_param(inst, "time", "0.33");
    int tlen = v2_get_param(inst, "time", buf, sizeof(buf));
    CHECK(tlen > 0 && fabsf((float)atof(buf) - 0.33f) < 1e-3f, "time is gettable (returns att_time)");

    v2_destroy_instance(inst);
}

static void test_chain_params(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
    char buf[8192];
    int len = v2_get_param(inst, "chain_params", buf, sizeof(buf));
    /* Master/Send FX bus reads metadata from here; must be a non-empty array
     * carrying the new labels + correct ranges/units (not the stale cache). */
    CHECK(len > 2 && buf[0] == '[' && buf[len - 1] == ']', "chain_params is a JSON array");
    CHECK(strstr(buf, "\"name\":\"Low Up Thr\"") != NULL, "chain_params has clear labels");
    CHECK(strstr(buf, "\"key\":\"low_cross\",\"name\":\"Low/Mid Hz\",\"type\":\"float\",\"min\":20,\"max\":18000") != NULL,
          "chain_params carries real ranges/units (crossover in Hz)");
    CHECK(strstr(buf, "\"unit\":\"dB\"") != NULL && strstr(buf, "\"unit\":\"Hz\"") != NULL, "units present");
    v2_destroy_instance(inst);
}

/* vitOTTx wraps every DSP-facing param in a 5 Hz SmoothValue; the port must
 * ramp rather than jump, and must still converge exactly on the target. */
static void test_smoothing(void) {
    ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);

    /* A fresh instance starts AT its defaults, not ramping up from zero. */
    CHECK_NEAR(inst->sm_use[SM_MGAIN], 11.7f, 1e-3f, "smoothers snap to defaults on create");

    float in[OTTX_MAXBLK] = {0}, out[OTTX_MAXBLK];
    v2_set_param(inst, "mgain", "0");

    /* set_param alone must not move the value the DSP reads... */
    CHECK_NEAR(inst->sm_use[SM_MGAIN], 11.7f, 1e-3f, "set_param does not jump the smoothed value");

    /* ...and one block must only take a fraction of the way there. */
    ottx_process_channel(inst, 0, in, out, 128);
    CHECK(inst->sm_use[SM_MGAIN] > 10.0f && inst->sm_use[SM_MGAIN] < 11.7f,
          "one block ramps partway toward the new target");

    /* ~30 ms (a few hundred blocks) later it has landed exactly. */
    for (int blk = 0; blk < 400; blk++) ottx_process_channel(inst, 0, in, out, 128);
    CHECK(inst->sm_use[SM_MGAIN] == 0.0f, "smoother converges exactly on the target");

    /* Crossover coefficients track the SMOOTHED frequency, not the raw param. */
    v2_set_param(inst, "low_cross", "1000");
    ottx_process_channel(inst, 0, in, out, 128);
    CHECK(inst->c_low.cutoff > 120.0f && inst->c_low.cutoff < 1000.0f,
          "crossover coefficients follow the smoothed frequency");
    for (int blk = 0; blk < 400; blk++) ottx_process_channel(inst, 0, in, out, 128);
    CHECK_NEAR(inst->c_low.cutoff, 1000.0f, 1e-2f, "crossover lands on the target frequency");

    /* A patch recall replaces everything at once — that must not smear. */
    v2_set_param(inst, "state", "{\"mgain\":5.0,\"low_cross\":300.0}");
    CHECK_NEAR(inst->sm_use[SM_MGAIN], 5.0f, 1e-3f, "state restore snaps (no ramp across a recall)");
    CHECK_NEAR(inst->c_low.cutoff, 300.0f, 1e-2f, "state restore snaps the crossover too");

    v2_destroy_instance(inst);
}

/* MultibandCompressor::processWithInput has four branches; a crossover pinned
 * to one end drops a band rather than filtering at 20 Hz / 18 kHz. */
static void test_collapse_modes(void) {
    struct { const char *lc, *hc; int mode; const char *label; } cases[] = {
        {"120", "2500",  OTTX_MODE_MULTI,  "120/2500 -> kMultiband"},
        {"120", "18000", OTTX_MODE_LOW,    "mid/high pinned -> kLowBand"},
        {"20",  "2500",  OTTX_MODE_HIGH,   "low/mid pinned -> kHighBand"},
        {"20",  "18000", OTTX_MODE_SINGLE, "both pinned -> kSingleBand"},
    };
    for (unsigned c = 0; c < sizeof(cases) / sizeof(*cases); c++) {
        ottx_t *inst = (ottx_t *)v2_create_instance(NULL, NULL);
        v2_set_param(inst, "low_cross", cases[c].lc);
        v2_set_param(inst, "high_cross", cases[c].hc);
        CHECK(inst->mode == cases[c].mode, cases[c].label);

        /* every branch must stay finite and actually pass signal */
        float in[OTTX_MAXBLK], out[OTTX_MAXBLK];
        int finite = 1; double oe = 0;
        for (int blk = 0; blk < 400; blk++) {
            for (int i = 0; i < 64; i++) {
                float t = (blk * 64 + i) / OTTX_SR;
                in[i] = 0.15f * (sinf(2*(float)M_PI*60*t) + sinf(2*(float)M_PI*900*t)
                               + sinf(2*(float)M_PI*6000*t)) / 3.0f;
            }
            ottx_process_channel(inst, 0, in, out, 64);
            for (int i = 0; i < 64; i++) { if (!isfinite(out[i])) finite = 0; if (blk > 200) oe += out[i]*out[i]; }
        }
        CHECK(finite && oe > 0.0, "collapse branch is finite and passes audio");
        v2_destroy_instance(inst);
    }
}

int main(void) {
    test_fastmath();
    test_smoothing();
    test_collapse_modes();
    test_lr_filter();
    test_passthrough_when_dry();
    test_bypass();
    test_compressor();
    test_multiband();
    test_params();
    test_ui_hierarchy_submenu();
    test_chain_params();
    printf(g_fail ? "\nSOME TESTS FAILED\n" : "\nALL TESTS PASSED\n");
    return g_fail;
}
