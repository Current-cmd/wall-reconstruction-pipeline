# Docker Setup for Wall Reconstruction Pipeline

Run the wall reconstruction pipeline in an isolated Docker container with all dependencies pre-installed.

## Quick Start

### 1. Build the Docker Image

```bash
docker build -t wall-reconstruction:latest .
```

### 2. Run with Docker Compose

Process Building 2nd Floor:
```bash
docker-compose up
```

Edit `docker-compose.yml` to change the input directory or processing mode.

### 3. Run with Shell Script (Recommended)

**Batch Mode** (process all files):
```bash
chmod +x docker-run.sh
./docker-run.sh --build --batch-mode ./raw_pointCloud/Building_2_2nd_Floor
```

**File Watcher Mode** (monitor for new files):
```bash
./docker-run.sh --watch-mode --interval 3 ./raw_pointCloud/clips
```

## Manual Docker Commands

### Batch Mode
```bash
docker run --rm \
  -v $(pwd)/raw_pointCloud/Building_2_2nd_Floor:/app/input:ro \
  -v $(pwd)/docker_output:/app/output \
  -v $(pwd):/app/work \
  -w /app/work \
  wall-reconstruction:latest \
  /bin/bash -c "source activate open3d_project_env && \
    python run_corridor_incremental.py --root /app/work --batches /app/input && \
    cp -r output_* /app/output/"
```

### File Watcher Mode
```bash
docker run --rm -it \
  -v $(pwd)/raw_pointCloud/clips:/app/input \
  -v $(pwd)/docker_output:/app/output \
  -v $(pwd):/app/work \
  -w /app/work \
  wall-reconstruction:latest \
  /bin/bash -c "source activate open3d_project_env && \
    python run_corridor_incremental.py --root /app/work --watch /app/input --interval 5 && \
    cp -r output_* /app/output/"
```

## Volume Mounts

| Host Path | Container Path | Purpose | Mode |
|-----------|---------------|---------|------|
| `./raw_pointCloud/...` | `/app/input` | Input point clouds | Read-only |
| `./docker_output` | `/app/output` | Processing results | Read-write |
| `$(pwd)` | `/app/work` | Pipeline scripts & workspace | Read-write |

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `MPLBACKEND` | `Agg` | Matplotlib backend (headless) |

## Resource Limits

Configure in `docker-compose.yml`:
```yaml
deploy:
  resources:
    limits:
      cpus: '4'
      memory: 8G
```

## Output Structure

Results are saved to `./docker_output/`:
```
docker_output/
└── output_Building_2_2nd_Floor/
    ├── batch_01_*/
    ├── batch_02_*/
    ├── walls_final.ifc
    └── visualizations/
        ├── final_top_view.png
        ├── final_side_view.png
        ├── final_front_view.png
        └── point_cloud_stats.txt
```

## Troubleshooting

### Permission Issues
If output files have wrong permissions:
```bash
sudo chown -R $USER:$USER docker_output/
```

### Out of Memory
Increase memory limit in docker-compose.yml or add to docker run:
```bash
docker run --memory=8g --memory-swap=8g ...
```

### Display Issues (Visualization)
The container uses headless matplotlib (Agg backend). No display needed.

### View Container Logs
```bash
docker logs wall-reconstruction
```

### Interactive Shell
```bash
docker run --rm -it \
  -v $(pwd):/app/work \
  -w /app/work \
  wall-reconstruction:latest \
  /bin/bash
```

Then inside container:
```bash
source activate open3d_project_env
python run_corridor_incremental.py --help
```

## Building for Production

### Multi-stage Build (Smaller Image)
```dockerfile
# Add to Dockerfile for production
FROM continuumio/miniconda3:latest as builder
# ... build steps ...

FROM continuumio/miniconda3:latest
COPY --from=builder /opt/conda /opt/conda
# ... runtime config ...
```

### Push to Registry
```bash
docker tag wall-reconstruction:latest myregistry/wall-reconstruction:v1.0
docker push myregistry/wall-reconstruction:v1.0
```

## Integration with CI/CD

### GitHub Actions Example
```yaml
name: Process Point Clouds
on: [push]
jobs:
  process:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build Docker image
        run: docker build -t wall-reconstruction .
      - name: Process data
        run: |
          docker run --rm \
            -v ${{ github.workspace }}/data:/app/input:ro \
            -v ${{ github.workspace }}/output:/app/output \
            wall-reconstruction
```

## Notes

- Input directory is mounted read-only (`:ro`) for safety
- Output is copied from container to host after processing
- Scripts are mounted read-only to prevent accidental modifications
- Container runs as root; adjust with `--user` flag if needed
