#!/usr/bin/env bash
# Native host-side unit tests for the OTTx DSP (no device/cross-compiler needed).
set -e
cd "$(dirname "$0")/.."
gcc -O2 -DOTTX_TEST -Isrc/dsp tests/test_ottx.c -o /tmp/ottx_test -lm
/tmp/ottx_test
