# Vertex AI 单节点 8卡 GPU 自定义训练作业指南

本目录（`b/gcp/demo1/`）包含在 Google Cloud Vertex AI 上使用单个节点运行 8卡 H200 GPU 训练任务的完整配置文件和提交脚本。该方案通过 **声明式预留亲和性（Reservation Affinity）**，将计算资源精确定位到了您在 `europe-west4-a` 可用区的专属硬件预留上，同时采用了 **无 Docker 本地打包（Python Source Distribution）** 的轻量提交流程。

---

## 目录结构

```text
b/gcp/demo1/
├── README.md         # 本说明文档
├── config.yaml       # Vertex AI CustomJobSpec 作业配置文件
├── commit.sh         # 本地打包、上传 GCS 并提交作业的自动化脚本
├── setup.py          # Python 打包标准配置文件
└── trainer/          # 训练代码包
    ├── __init__.py   # 模块初始化文件
    └── task.py       # 训练入口脚本（打印检测到的 GPU 硬件信息）
```

---

## 核心配置详解

### 1. 训练入口代码：`trainer/task.py`
该脚本在运行时会加载 PyTorch，检测当前节点被分配的 GPU 数量，并打印出每张 GPU 的型号。

```python
import torch
import os
import sys

def main():
    # 强制将 stdout 重新配置为行缓冲 (遇到 \n 即刷新)，保证日志零延迟
    sys.stdout.reconfigure(line_buffering=True)
    
    world_size = int(os.environ.get("WORLD_SIZE", 1))
    rank = int(os.environ.get("RANK", 0))
    gpu_count = torch.cuda.device_count()
    
    print(f"--- Node Rank: {rank}/{world_size} ---")
    print(f"Detected GPUs on this node: {gpu_count}")
    
    for i in range(gpu_count):
        print(f"GPU {i}: {torch.cuda.get_device_name(i)}")

if __name__ == "__main__":
    main()
```

### 2. 声明式硬件锁定：`config.yaml`
此文件定义了单节点（`replicaCount: 1`）、8张 H200 GPU 的规格。为确保资源落在 `europe-west4-a` 的专属硬件预留中，我们配置了 `reservationAffinity`：

```yaml
workerPoolSpecs:
  - machineSpec:
      machineType: a3-ultragpu-8g
      acceleratorType: NVIDIA_H200_141GB
      acceleratorCount: 8
      reservationAffinity:
        reservationAffinityType: SPECIFIC_RESERVATION  # 指定使用特定硬件预留（Specific Reservation）
        key: compute.googleapis.com/reservation-name
        values: # 指向 europe-west4-a 下的 H200 硬件预留资源路径
          - projects/autel-ai-physical-spat-intel/zones/europe-west4-a/reservations/reservation-20260422-033135
    replicaCount: 1
    pythonPackageSpec:
      # 使用欧洲（europe-west4）官方免拉取的高性能预构建 PyTorch 2.4 GPU 镜像
      executorImageUri: europe-docker.pkg.dev/vertex-ai/training/pytorch-gpu.2-4.py310:latest
      packageUris:
        - gs://physical-ai-data-eu/demo1/trainer-0.1.tar.gz
      pythonModule: trainer.task
      env:
        - name: PYTHONUNBUFFERED
          value: "1"

scheduling:
  timeout: 604800s
```

### 3. 一键提交流程脚本：`commit.sh`
传统的 `gcloud` 提交流程如果使用 `local-package-path` 参数，会强制要求本地安装并启动 Docker 引擎。为了免去本地 Docker 依赖，本方案通过下述三步轻松完成：

```bash
#!/bin/bash
set -e

# 1. 在本地无 Docker 环境下打包 Python 源码包
echo "Packaging training code..."
python3 setup.py sdist --formats=gztar

# 2. 将打包好的 tar.gz 上传到具有写权限的 GCS 桶中
echo "Uploading package to GCS..."
gsutil cp dist/trainer-0.1.tar.gz gs://physical-ai-data-eu/demo1/trainer-0.1.tar.gz

# 3. 读取 config.yaml 声明，向 Vertex AI 提交单节点训练任务
echo "Submitting single-node training job to Vertex AI..."
gcloud ai custom-jobs create \
    --region=europe-west4 \
    --display-name=bttest \
    --config=config.yaml
```

---

## 部署与操作步骤

请按照以下步骤，从命令行打包并一键提交您的训练作业：

### 第一步：切换到工作目录
```bash
cd /home/physical/SRC/RL/RLinf/b/gcp/demo1
```

### 第二步：运行一键提交脚本
```bash
bash commit.sh
```

**运行输出范例**：
```text
Packaging training code...
running sdist
...
Creating tar archive
Uploading package to GCS...
Copying file://dist/trainer-0.1.tar.gz [Content-Type=application/x-tar]...
Operation completed over 1 objects/1.1 KiB.                                      
Submitting single-node training job to Vertex AI...
Using endpoint [https://europe-west4-aiplatform.googleapis.com/]
CustomJob [projects/73851708908/locations/europe-west4/customJobs/3931099178511368192] is submitted successfully.
```

---

## 跟踪作业状态与日志

作业提交成功后，您可以通过以下几种方式查看状态与实时日志：

### 1. 通过 GCP 控制台（Web 浏览器）
*   **直达本作业详情页（推荐）**：
    [直达当前作业详情页](https://console.cloud.google.com/vertex-ai/training/custom-jobs/locations/europe-west4/custom-jobs/3931099178511368192?project=autel-ai-physical-spat-intel)
*   **直达作业输出日志页面**：
    [直达 Cloud Logging 日志流](https://console.cloud.google.com/logs/query;query=resource.type%3D%22ml_job%22%20AND%20resource.labels.job_id%3D%223931099178511368192%22?project=autel-ai-physical-spat-intel)
*   **查看所有自定义作业列表**：
    [查看自定义作业列表](https://console.cloud.google.com/vertex-ai/training/custom-jobs?project=autel-ai-physical-spat-intel)

### 2. 通过 gcloud 命令行（CLI 终端）
*   **查询作业状态与元数据**：
    ```bash
    gcloud ai custom-jobs describe projects/73851708908/locations/europe-west4/customJobs/3931099178511368192
    ```
*   **在本地终端实时流式监听训练输出**：
    ```bash
    gcloud ai custom-jobs stream-logs projects/73851708908/locations/europe-west4/customJobs/3931099178511368192
    ```
