#!/bin/bash

# Docker run script for wall reconstruction pipeline
# Usage: ./docker-run.sh [--build] [--batch-mode|--watch-mode] <input_directory>

set -e

# Parse arguments
BUILD=false
MODE="batch"
INPUT_DIR=""
INTERVAL=5

while [[ $# -gt 0 ]]; do
  case $1 in
    --build)
      BUILD=true
      shift
      ;;
    --batch-mode)
      MODE="batch"
      shift
      ;;
    --watch-mode)
      MODE="watch"
      shift
      ;;
    --interval)
      INTERVAL="$2"
      shift 2
      ;;
    *)
      INPUT_DIR="$1"
      shift
      ;;
  esac
done

# Validate input directory
if [ -z "$INPUT_DIR" ]; then
  echo "Usage: $0 [--build] [--batch-mode|--watch-mode] [--interval N] <input_directory>"
  echo ""
  echo "Options:"
  echo "  --build           Build Docker image before running"
  echo "  --batch-mode      Process all files at once (default)"
  echo "  --watch-mode      Monitor directory for new files"
  echo "  --interval N      Poll interval for watch mode (default: 5 seconds)"
  echo ""
  echo "Examples:"
  echo "  $0 --build --batch-mode ./raw_pointCloud/Building_2_2nd_Floor"
  echo "  $0 --watch-mode --interval 3 ./raw_pointCloud/clips"
  exit 1
fi

if [ ! -d "$INPUT_DIR" ]; then
  echo "Error: Input directory does not exist: $INPUT_DIR"
  exit 1
fi

# Build image if requested
if [ "$BUILD" = true ]; then
  echo "Building Docker image..."
  docker build -t wall-reconstruction:latest .
fi

# Create output directory
OUTPUT_DIR="./docker_output"
mkdir -p "$OUTPUT_DIR"

# Get absolute paths
INPUT_ABS=$(cd "$INPUT_DIR" && pwd)
OUTPUT_ABS=$(cd "$OUTPUT_DIR" && pwd)
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

echo "================================================"
echo "Wall Reconstruction Pipeline - Docker Mode"
echo "================================================"
echo "Mode: $MODE"
echo "Input: $INPUT_ABS"
echo "Output: $OUTPUT_ABS"
if [ "$MODE" = "watch" ]; then
  echo "Interval: ${INTERVAL}s"
fi
echo "================================================"

# Run container
if [ "$MODE" = "batch" ]; then
  docker run --rm \
    -v "$INPUT_ABS:/app/input:ro" \
    -v "$OUTPUT_ABS:/app/output" \
    -v "$SCRIPT_DIR:/app/work" \
    -w /app/work \
    wall-reconstruction:latest \
    /bin/bash -c "
      source activate open3d_project_env && 
      python run_corridor_incremental.py --root /app/work --batches /app/input && 
      if ls output_* 1> /dev/null 2>&1; then
        cp -r output_* /app/output/ && 
        echo 'Results copied to /app/output/'
      else
        echo 'Warning: No output directories found'
      fi
    "
else
  echo "Starting file watcher mode... Press Ctrl+C to stop and finalize."
  docker run --rm -it \
    -v "$INPUT_ABS:/app/input" \
    -v "$OUTPUT_ABS:/app/output" \
    -v "$SCRIPT_DIR:/app/work" \
    -w /app/work \
    wall-reconstruction:latest \
    /bin/bash -c "
      source activate open3d_project_env && 
      python run_corridor_incremental.py --root /app/work --watch /app/input --interval $INTERVAL && 
      if ls output_* 1> /dev/null 2>&1; then
        cp -r output_* /app/output/ && 
        echo 'Results copied to /app/output/'
      else
        echo 'Warning: No output directories found'
      fi
    "
fi

echo ""
echo "================================================"
echo "Processing complete!"
echo "Results saved to: $OUTPUT_ABS"
echo "================================================"
