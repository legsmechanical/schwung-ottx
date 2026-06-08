#!/usr/bin/env bash
# Build OTTx module for Schwung (ARM64)
#
# Automatically uses Docker for cross-compilation if needed.
# Set CROSS_PREFIX to skip Docker (e.g., for native ARM builds).
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="schwung-builder"

# Check if we need Docker
if [ -z "$CROSS_PREFIX" ] && [ ! -f "/.dockerenv" ]; then
    echo "=== OTTx Module Build (via Docker) ==="
    echo ""

    # Build Docker image if needed
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo "Building Docker image (first time only)..."
        docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
        echo ""
    fi

    # Run build inside container
    echo "Running build..."
    docker run --rm \
        -v "$REPO_ROOT:/build" \
        -u "$(id -u):$(id -g)" \
        -w /build \
        "$IMAGE_NAME" \
        ./scripts/build.sh

    echo ""
    echo "=== Done ==="
    exit 0
fi

# === Actual build (runs in Docker or with cross-compiler) ===
CROSS_PREFIX="${CROSS_PREFIX:-aarch64-linux-gnu-}"

cd "$REPO_ROOT"

echo "=== Building OTTx Module ==="
echo "Cross prefix: $CROSS_PREFIX"

# Create build directories
mkdir -p build
mkdir -p dist/ottx

# Compile DSP plugin.
# NOTE: -O3, NOT -Ofast. OTTx ports Vital's polynomial fast-math which uses
# float bit-manipulation and relies on bounded Inf behavior (the lenv=0 path
# is intentionally clamped by MAX_EXPAND); -ffast-math would break those
# IEEE-754 assumptions, so it is deliberately avoided.
echo "Compiling DSP plugin..."
${CROSS_PREFIX}gcc -O3 -shared -fPIC \
    -DNDEBUG \
    src/dsp/ottx.c \
    -o build/ottx.so \
    -Isrc/dsp \
    -lm

# Copy files to dist (use cat to avoid ExtFS deallocation issues with Docker).
# The shared library MUST be named <id>.so (ottx.so) — the chain host loads
# audio FX as modules/audio_fx/<id>/<id>.so, ignoring module.json's dsp field.
echo "Packaging..."
cat src/module.json > dist/ottx/module.json
[ -f src/help.json ]   && cat src/help.json   > dist/ottx/help.json
[ -f src/ui_chain.js ] && cat src/ui_chain.js > dist/ottx/ui_chain.js
[ -f LICENSE ]         && cat LICENSE         > dist/ottx/LICENSE
cat build/ottx.so > dist/ottx/ottx.so
chmod +x dist/ottx/ottx.so

# Create tarball for release
cd dist
tar -czvf ottx-module.tar.gz ottx/
cd ..

echo ""
echo "=== Build Complete ==="
echo "Output: dist/ottx/"
echo "Tarball: dist/ottx-module.tar.gz"
echo ""
echo "To install on Move:"
echo "  ./scripts/install.sh"
