# Makefile for docker-compose workflow

# .PHONY 告诉 make，这些目标不是文件名，而是要执行的命令
.PHONY: build up run down stop shell logs clean help

# 默认目标，当你只输入 make 时执行
default: help

build:
	@echo "--- 正在构建或更新Docker镜像... ---"
	@docker compose build

run:
	@echo "--- 正在后台启动开发环境... ---"
	@xhost +local:
	@docker compose up -d

stop:
	@echo "--- 正在停止并移除开发环境... ---"
	@docker compose down

shell:
	@echo "--- 正在进入容器的交互式终端... ---"
	@docker compose exec rm_robot_dev bash

logs:
	@echo "--- 正在查看容器日志... ---"
	@docker compose logs -f

clean:
	@echo "--- 正在清理已停止的容器... ---"
	@docker container prune -f

help:
	@echo "可用命令:"
	@echo "  make build   - 构建或更新Docker镜像 (修改Dockerfile后运行)"
	@echo "  make run     - 在后台启动开发环境"
	@echo "  make stop    - 停止并移除开发环境 (会清空编译缓存)"
	@echo "  make shell   - 进入正在运行的容器终端"
	@echo "  make logs    - 实时查看容器的输出日志"
	@echo "  make clean   - 清理系统中所有已停止的容器"
