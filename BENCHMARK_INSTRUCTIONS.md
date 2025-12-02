# Benchmarking Instructions

## Quick Start with Docker (Recommended)

1. **Install Docker Desktop**
   - Download from https://www.docker.com/products/docker-desktop

2. **Build the image** (one-time setup):
   ```bash
   docker build -t wall-reconstruction:latest .
   ```

3. **Run benchmarks on your point cloud data**:
   ```bash
   # Single dataset
   ./docker-run.sh --batch-mode /path/to/your/pointcloud/folder

   # Time it
   time ./docker-run.sh --batch-mode /path/to/your/pointcloud/folder
   ```

4. **Results**:
   - Processing logs show timing for each batch
   - Check `docker_output/` for results
   - Final summary shows total walls detected and time

## Manual Setup (Alternative)

1. **Create conda environment**:
   ```bash
   conda create -n wall_pipeline python=3.9 -y
   conda activate wall_pipeline
   ```

2. **Install dependencies**:
   ```bash
   pip install open3d numpy scipy ifcopenshell laspy matplotlib pandas scikit-learn
   ```

3. **Run benchmark**:
   ```bash
   time python run_corridor_incremental.py \
     --root . \
     --batches /path/to/your/pointcloud/folder
   ```

## Benchmark Metrics to Record

- **Total processing time** (from start to finish)
- **Number of point cloud files processed**
- **Total points processed**
- **Number of walls detected**
- **Time per batch** (shown in logs)
- **System specs**: CPU, RAM, OS

## Example Output

```
[TIME] Batch 1/12: 45.2 seconds
[TIME] Total processing: 8 minutes 39 seconds
[INFO] Total walls detected: 211
```

## Test Datasets

Recommended testing:
1. Small dataset (1-3 files) - Quick validation
2. Medium dataset (12 files) - Standard benchmark  
3. Large dataset (20+ files) - Stress test

## Troubleshooting

- **Docker:** Ensure Docker Desktop is running
- **Memory:** Pipeline needs ~8GB RAM for large datasets
- **Permissions:** `chmod +x docker-run.sh` if needed

## Support

For issues or questions, please open an issue on the GitHub repository.
