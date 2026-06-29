#!/usr/bin/env bash

# Source this file in every terminal before running the grasp workspace.
# Usage:
#   source /data/jiaxuanLin/grasp_ws/env.sh

export GRASP_WS=/data/jiaxuanLin/grasp_ws
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"

source /opt/ros/jazzy/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate /data/jiaxuanLin/conda_envs/grasp_ws_py312

if [ -f "$GRASP_WS/install/setup.bash" ]; then
  source "$GRASP_WS/install/setup.bash"
fi

cd "$GRASP_WS"
