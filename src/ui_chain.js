/*
 * OTTx Chain UI
 *
 * Simple parameter editor for OTTx when editing in chain mode.
 * Uses globalThis.chain_ui pattern for loading inside shadow UI.
 *
 * This is a quick 4-knob editor for the headline OTT macros; the full
 * parameter set (per-band thresholds/ratios/gains, crossovers) is reachable
 * via the auto-generated shadow UI from module.json's ui_hierarchy.
 */

import {
    MoveMainKnob,
    MoveKnob1, MoveKnob2, MoveKnob3, MoveKnob4
} from '/data/UserData/schwung/shared/constants.mjs';

import { decodeDelta } from '/data/UserData/schwung/shared/input_filter.mjs';

import {
    drawMenuHeader as drawHeader,
    drawMenuFooter as drawFooter
} from '/data/UserData/schwung/shared/menu_layout.mjs';

/* Display constants */
const SCREEN_WIDTH = 128;
const SCREEN_HEIGHT = 64;

/* Headline OTT macros (mapped to Knobs 1-4). Each carries a display formatter
 * because OTTx mixes normalized (depth/mix) and ratio (upward/downward) ranges. */
const PCT   = (v) => `${Math.round(v * 100)}%`;
const RATIO = (v) => `${v.toFixed(2)}x`;

const PARAMS = [
    { key: "depth",    name: "Depth",    min: 0, max: 1, step: 0.02, fmt: PCT },
    { key: "upward",   name: "Upward",   min: 0, max: 2, step: 0.02, fmt: RATIO },
    { key: "downward", name: "Downward", min: 0, max: 2, step: 0.02, fmt: RATIO },
    { key: "mix",      name: "Mix",      min: 0, max: 1, step: 0.02, fmt: PCT }
];

/* State */
let selectedParam = 0;
let paramValues = [1.0, 1.0, 1.0, 1.0];  /* Defaults (match ottx_set_defaults) */
let needsRedraw = true;

/* Fetch current parameter values from DSP */
function fetchParams() {
    for (let i = 0; i < PARAMS.length; i++) {
        const val = host_module_get_param(PARAMS[i].key);
        if (val !== null && val !== undefined) {
            paramValues[i] = parseFloat(val) || PARAMS[i].min;
        }
    }
}

/* Set a parameter value */
function setParam(index, value) {
    const param = PARAMS[index];
    value = Math.max(param.min, Math.min(param.max, value));
    paramValues[index] = value;
    host_module_set_param(param.key, value.toFixed(3));
}

/* Adjust a parameter by delta */
function adjustParam(index, delta) {
    const param = PARAMS[index];
    const newVal = paramValues[index] + delta * param.step;
    setParam(index, newVal);
}

/* Draw the UI */
function drawUI() {
    clear_screen();
    drawHeader("OTTx");

    const listY = 16;
    const lineHeight = 11;

    for (let i = 0; i < PARAMS.length; i++) {
        const y = listY + i * lineHeight;
        const param = PARAMS[i];
        const isSelected = i === selectedParam;

        if (isSelected) {
            fill_rect(0, y - 1, SCREEN_WIDTH, lineHeight, 1);
        }

        const color = isSelected ? 0 : 1;
        const prefix = isSelected ? "> " : "  ";

        /* Show parameter name */
        print(2, y, `${prefix}${param.name}`, color);

        /* Show value using the param's formatter */
        const valueStr = param.fmt(paramValues[i]);
        print(SCREEN_WIDTH - valueStr.length * 6 - 4, y, valueStr, color);
    }

    drawFooter({left: "Jog: select", right: "Knobs: adjust"});
    needsRedraw = false;
}

/* Initialize */
function init() {
    fetchParams();
    needsRedraw = true;
}

/* Tick - called every frame */
function tick() {
    if (needsRedraw) {
        drawUI();
    }
}

/* Handle MIDI input */
function onMidiMessageInternal(data) {
    const status = data[0];
    const d1 = data[1];
    const d2 = data[2];

    /* Handle CC messages */
    if ((status & 0xF0) === 0xB0) {
        /* Jog wheel - select parameter */
        if (d1 === MoveMainKnob) {
            const delta = decodeDelta(d2);
            if (delta !== 0) {
                selectedParam = Math.max(0, Math.min(PARAMS.length - 1, selectedParam + delta));
                needsRedraw = true;
            }
            return;
        }

        /* Knobs 1-4 adjust corresponding parameters */
        if (d1 >= MoveKnob1 && d1 <= MoveKnob4) {
            const knobIndex = d1 - MoveKnob1;
            const delta = decodeDelta(d2);
            if (delta !== 0 && knobIndex < PARAMS.length) {
                adjustParam(knobIndex, delta);
                needsRedraw = true;
            }
            return;
        }
    }
}

/* Export as chain_ui for loading by shadow UI */
globalThis.chain_ui = {
    init,
    tick,
    onMidiMessageInternal
};
