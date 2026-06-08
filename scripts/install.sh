#!/bin/bash
# Install OTTx module to Move
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

if [ ! -d "dist/ottx" ]; then
    echo "Error: dist/ottx not found. Run ./scripts/build.sh first."
    exit 1
fi

echo "=== Installing OTTx Module ==="

# Deploy to Move - audio_fx subdirectory
echo "Copying module to Move..."
ssh ableton@move.local "mkdir -p /data/UserData/schwung/modules/audio_fx/ottx"
scp -r dist/ottx/* ableton@move.local:/data/UserData/schwung/modules/audio_fx/ottx/

# Set permissions so Module Store can update later
echo "Setting permissions..."
ssh ableton@move.local "chmod -R a+rw /data/UserData/schwung/modules/audio_fx/ottx"

echo ""
echo "=== Install Complete ==="
echo "Module installed to: /data/UserData/schwung/modules/audio_fx/ottx/"
echo ""
echo "Restart Schwung to load the new module."
