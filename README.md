# Schwung OTTx

A 3-band multiband **upward + downward** compressor audio-FX module for
[Schwung](https://github.com/charlesvestal/schwung) on the Ableton Move — the
classic "OTT" sound. This is a faithful scalar-C port of
[vitOTTx](https://github.com/Sakhnovkrg/vitOTTx), which itself builds on
[Vital](https://github.com/mtytel/vital)'s OTT compressor DSP.

## Prerequisites

- [Schwung](https://github.com/charlesvestal/schwung) (host **0.3.0+**) installed on your Ableton Move

## Installation

### Via Module Store (Recommended)

1. Launch Schwung on your Move
2. Select **Module Store** from the main menu
3. Navigate to **Audio FX** → **OTTx**
4. Select **Install**

### Manual Installation

```bash
./scripts/build.sh      # cross-compile for ARM64 via Docker
./scripts/install.sh    # deploy to move.local over SSH
```

## How it works

The signal is split into low/mid/high bands with 4th-order Linkwitz-Riley
crossovers, and each band is processed by an independent upward + downward
RMS compressor (the Vital algorithm). "Upward" brings quiet detail up toward
the band's lower threshold; "downward" tames peaks above the upper threshold —
together producing OTT's dense, in-your-face character. The bands are then
recombined with per-band makeup gain and a global dry/wet mix.

## Controls

Root knobs (the OTT macros):

| Knob | Range | Default | Description |
|------|-------|---------|-------------|
| Depth    | 0–100% | 100% | Scales all band compression ratios at once — the master "amount" |
| Upward   | 0–2×   | 1.0  | Multiplies the upward (lower) ratios |
| Downward | 0–2×   | 1.0  | Multiplies the downward (upper) ratios |
| Time     | 0–1    | 0.5  | Attack/release macro (drives both together) |
| In Gain  | −60…30 dB | 0 dB | Input trim |
| Out Gain | −60…30 dB | 0 dB | Output trim |
| Mix      | 0–100% | 100% | Dry/wet blend |

The **Advanced** level exposes the full per-band controls (lower/upper
thresholds and ratios, makeup gain for Low/Mid/High) plus both crossover
frequencies and the raw Attack/Release.

> **Note:** at Mix = 0% the dry signal is the crossover reconstruction (an
> allpass cascade): it is energy-preserving but phase-rotated, matching
> vitOTTx. For a true bit-exact bypass, use the module's **Bypass**.

## Building

```bash
./scripts/build.sh      # Build ottx.so for ARM64 via Docker, package the tarball
./scripts/install.sh    # Deploy to Move (move.local)
```

Releases are built automatically by CI on a `v*` tag (see
`.github/workflows/release.yml`) and published as `ottx-module.tar.gz`.

## Credits

- **OTT compressor DSP**: [Matt Tytel / Vital](https://github.com/mtytel/vital) (GPLv3)
- **vitOTT / vitOTTx**: [Yegor Suslin](https://github.com/edgjj/vitOTT) and [Sakhnovkrg](https://github.com/Sakhnovkrg/vitOTTx) (GPLv3)
- **Schwung port**: legsmechanical

## License

**GPLv3** — see the `LICENSE` file. Because the DSP derives from Vital (GPLv3),
this module and its binaries are distributed under the GPLv3.

## AI Assistance Disclaimer

This module was developed with AI assistance (Claude). All architecture,
implementation, and release decisions are reviewed by a human maintainer.
AI-assisted content may still contain errors, so please validate functionality,
security, and license compatibility before production use.
