# OTTx

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
2. Install via Schwung manager: http://move.local:7700/

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

**Noise floor (the one DSP difference from vitOTTx).** The Move's audio is
16-bit, so reverb tails and other residue settle at ±1 LSB instead of
decaying to true silence the way they do in a DAW. Unchecked, upward
compression would lift that to about −46 dBFS of high-end hiss whenever
nothing is playing. OTTx fades the upward gain out as a band approaches the
16-bit floor (from 18 dB above it down to 6 dB above it), and the floor
follows **In Gain**. Anything louder than that is processed exactly as
upstream. **Tame** moves the floor. Raise it by up to 12 dB for sources that
carry their own hiss, such as lo-fi reverbs and bitcrushed or tape-style
material, which would otherwise be pumped up into crackle. Lower it for
more upward gain on very quiet material. At −12 dB the floor is switched
off entirely, which is bit-identical to stock vitOTTx.

## Controls

### Root (the OTT macros)

These are mapped to the knobs, in this order:

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| Mix        | 0–100%       | 100%    | Dry/wet blend |
| Depth      | 0–100%       | 100%    | Scales every band's compression ratios at once — the master "amount" |
| Upward     | 0–2×         | 1.0×    | Multiplies the upward (boost-quiet) ratios on every band |
| Downward   | 0–2×         | 1.0×    | Multiplies the downward (tame-loud) ratios on every band |
| Time       | 0–100%       | 50%     | Attack/release macro — drives both together (higher = slower) |
| In Gain    | −60…+30 dB   | 0 dB    | Input trim |
| Out Gain   | −60…+30 dB   | 0 dB    | Output trim |
| Tame       | −12…+12 dB   | 0 dB    | Moves the noise floor: raise it so upward compression stops boosting a noisy source's own hiss (lo-fi reverbs, etc.); −12 turns the floor off entirely (stock vitOTTx behavior) |
| Low/Mid Hz | 20 Hz–18 kHz | 120 Hz  | Low ↔ mid crossover frequency |
| Mid/Hi Hz  | 20 Hz–18 kHz | 2.5 kHz | Mid ↔ high crossover frequency |

### Advanced (per-band detail)

Open **Advanced** from the bottom of the root list. It holds two global timing
controls plus five controls for each band (**Low / Mid / High**):

- **Up Thr** — the *upward* threshold. Signal **below** it is lifted up toward
  it; the quieter it is, the more it's boosted.
- **Up Ratio** — strength of that upward lift. Negative values flip it into
  downward *expansion* instead.
- **Dn Thr** — the *downward* threshold. Peaks **above** it are pushed back down
  toward it.
- **Dn Ratio** — strength of that downward compression.
- **Gain** — per-band makeup/output gain, applied after compression.

> Depth × Upward scales every band's **Up Ratio**, and Depth × Downward scales
> every **Dn Ratio** — the root macros ride on top of these per-band amounts.

| Param        | Range       | Default  | Description |
|--------------|-------------|----------|-------------|
| Attack       | 0–100%      | 50%      | Envelope attack for all bands — higher = slower reaction to transients |
| Release      | 0–100%      | 50%      | Envelope release — higher = slower recovery |
| Low Up Thr   | −80…0 dB    | −35 dB   | Low band — upward (boost) threshold |
| Low Up Ratio | −100…100%   | 80%      | Low band — upward amount (negative = expansion) |
| Low Dn Thr   | −80…0 dB    | −28 dB   | Low band — downward (tame) threshold |
| Low Dn Ratio | 0–100%      | 90%      | Low band — downward amount |
| Low Gain     | −30…+30 dB  | 16.3 dB  | Low band — makeup gain |
| Mid Up Thr   | −80…0 dB    | −36 dB   | Mid band — upward threshold |
| Mid Up Ratio | −100…100%   | 80%      | Mid band — upward amount |
| Mid Dn Thr   | −80…0 dB    | −25 dB   | Mid band — downward threshold |
| Mid Dn Ratio | 0–100%      | 86%      | Mid band — downward amount |
| Mid Gain     | −30…+30 dB  | 11.7 dB  | Mid band — makeup gain |
| Hi Up Thr    | −80…0 dB    | −35 dB   | High band — upward threshold |
| Hi Up Ratio  | −100…100%   | 80%      | High band — upward amount |
| Hi Dn Thr    | −80…0 dB    | −30 dB   | High band — downward threshold |
| Hi Dn Ratio  | 0–100%      | 100%     | High band — downward amount |
| Hi Gain      | −30…+30 dB  | 16.3 dB  | High band — makeup gain |

The defaults above are the classic OTT curve (inherited from vitOTTx).

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
