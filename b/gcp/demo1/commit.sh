#!/bin/bash
set -e

# 1. 自动在本地打包 Python 代码（完全不需要 Docker 引擎）
echo "Packaging training code..."
python3 setup.py sdist --formats=gztar

# 2. 将打包好的源码包上传至 Google Cloud Storage 桶
echo "Uploading package to GCS..."
gsutil cp dist/trainer-0.1.tar.gz gs://physical-ai-data-eu/demo1/trainer-0.1.tar.gz

# 3. 提交分布式自定义训练作业至 Vertex AI
echo "Submitting single-node training job to Vertex AI..."
gcloud ai custom-jobs create \
    --region=europe-west4 \
    --display-name=bttest \
    --config=config.yaml
