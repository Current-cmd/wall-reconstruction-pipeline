FROM continuumio/miniconda3:latest

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Create conda environment
RUN conda create -n open3d_project_env python=3.9 -y

# Activate environment and install Python packages
RUN echo "source activate open3d_project_env" > ~/.bashrc
ENV PATH /opt/conda/envs/open3d_project_env/bin:$PATH

# Install Python dependencies
RUN /opt/conda/envs/open3d_project_env/bin/pip install --no-cache-dir \
    open3d \
    numpy \
    scipy \
    scikit-learn \
    ifcopenshell \
    laspy \
    matplotlib \
    pandas

# Copy application files
COPY *.py /app/
COPY README.md /app/

# Create directories for input/output
RUN mkdir -p /app/input /app/output

# Set environment variable for display (needed for matplotlib)
ENV MPLBACKEND=Agg

# Set working directory with write permissions
WORKDIR /app/work

# Default command
CMD ["/bin/bash"]
