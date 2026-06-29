#!/bin/bash
# ==========================================================
#  ROS 2 项目自动化安装脚本 (智能版)
# ==========================================================
#
# 功能：
# 1. 自动安装 GraspNet 和 Grounded-SAM 的项目依赖。
# 2. 自动编译 C++/CUDA 扩展模块。
# 3. 自动下载所需的预训练模型。
# 4. 通过检查标记文件，自动跳过已完成的步骤，可以安全地重复运行。
#
# 用法：
# 在进入容器后，于 ros2_ws 目录下手动运行此脚本:
# ./setup_project.sh
#
# ==========================================================

# 如果任何命令失败，则立即退出脚本
set -e

echo "=========================================================="
echo "=== 智能安装与编译脚本启动... ==="
echo "=========================================================="

# --- 配置 ---
WORKSPACE_DIR="/root/host_home/ros2_ws"
LOCK_DIR="$WORKSPACE_DIR/.setup_locks"

# 创建一个隐藏目录来存放标记文件
mkdir -p "$LOCK_DIR"
echo "工作目录设置为: $WORKSPACE_DIR"
echo "标记文件目录设置为: $LOCK_DIR"
echo "----------------------------------------------------------"


# --- 1. 安装 GraspNet 的 Python 依赖 ---
if [ ! -f "$LOCK_DIR/graspnet_pip.lock" ]; then
    echo ">>> [1/5] 正在安装 GraspNet Python 依赖 (requirements.txt)..."
    source /opt/venv_graspnet/bin/activate
    pip install -r $WORKSPACE_DIR/graspnet-baseline/requirements.txt
    deactivate
    touch "$LOCK_DIR/graspnet_pip.lock"
    echo "✅ GraspNet Python 依赖安装完成。"
else
    echo ">>> [1/5] GraspNet Python 依赖已安装，跳过。"
fi
echo "----------------------------------------------------------"


# --- 2. 编译 GraspNet C++/CUDA 扩展 ---
if [ ! -f "$LOCK_DIR/graspnet_ext.lock" ]; then
    echo ">>> [2/5] 正在编译 GraspNet C++/CUDA 扩展..."
    source /opt/venv_graspnet/bin/activate
    cd $WORKSPACE_DIR/graspnet-baseline/pointnet2 && python setup.py install
    cd $WORKSPACE_DIR/graspnet-baseline/knn && python setup.py install
    cd $WORKSPACE_DIR
    pip install -e $WORKSPACE_DIR/graspnet-baseline/graspnetAPI
    deactivate
    touch "$LOCK_DIR/graspnet_ext.lock"
    echo "✅ GraspNet 扩展编译完成。"
else
    echo ">>> [2/5] GraspNet C++/CUDA 扩展已编译，跳过。"
fi
echo "----------------------------------------------------------"


# --- 3. 安装 Grounded-SAM-2 主项目 ---
if [ ! -f "$LOCK_DIR/groundedsam_main.lock" ]; then
    echo ">>> [3/5] 正在安装 Grounded-SAM-2 主项目..."
    source /opt/venv_groundedsam/bin/activate
    cd $WORKSPACE_DIR/Grounded-SAM-2
    pip install -e .
    cd $WORKSPACE_DIR
    deactivate
    touch "$LOCK_DIR/groundedsam_main.lock"
    echo "✅ Grounded-SAM-2 主项目安装完成。"
else
    echo ">>> [3/5] Grounded-SAM-2 主项目已安装，跳过。"
fi
echo "----------------------------------------------------------"


# --- 4. 编译 Grounding DINO 扩展 ---
if [ ! -f "$LOCK_DIR/groundedsam_ext.lock" ]; then
    echo ">>> [4/5] 正在编译 Grounding DINO 扩展..."
    source /opt/venv_groundedsam/bin/activate
    
    # 确认编译器版本
    export CC=gcc-12
    export CXX=g++-12
    
    # 先手动编译扩展
    cd $WORKSPACE_DIR/Grounded-SAM-2/grounding_dino
    python setup.py build_ext --inplace
    
    # 然后安装为 editable 模式
    cd $WORKSPACE_DIR/Grounded-SAM-2
    pip install --no-build-isolation --no-deps -e grounding_dino
    
    cd $WORKSPACE_DIR
    deactivate
    touch "$LOCK_DIR/groundedsam_ext.lock"
    echo "✅ Grounding DINO 扩展编译完成。"
else
    echo ">>> [4/5] Grounding DINO 扩展已编译，跳过。"
fi
echo "----------------------------------------------------------"


# --- 5. 下载模型权重 ---
SAM_CHECKPOINT_FILE="$WORKSPACE_DIR/Grounded-SAM-2/checkpoints/sam2_hiera_base_plus.pt"

if [ ! -f "$SAM_CHECKPOINT_FILE" ]; then
    echo ">>> [5/5] 正在下载或检查预训练模型..."
    cd $WORKSPACE_DIR/Grounded-SAM-2/checkpoints && bash download_ckpts.sh
    cd $WORKSPACE_DIR/Grounded-SAM-2/gdino_checkpoints && bash download_ckpts.sh
    cd $WORKSPACE_DIR
    echo "✅ 模型下载完成。"
else
    echo ">>> [5/5] 预训练模型已存在，跳过。"
fi
echo "----------------------------------------------------------"


echo "=========================================================="
echo "=== 所有项目设置已成功完成！ ==="
echo "=========================================================="