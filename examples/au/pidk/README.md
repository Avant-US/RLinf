# pi0.5 (openpi_au) 在 pushdoor 数据上的 Docker 容器化分布式 SFT 示例

本目录是 [`examples/au/pi/`](../pi/) 的 **Docker 容器化**版本：同样用 RLinf 的
`openpi_au` 版 pi0.5 模型在 R1-Pro "push door" LeRobot 数据集上做 SFT，但训练过程
运行在 `rlinf-aupi-dev:260702` Docker 容器内，并支持**多机分布式训练**。

- 模型实现：[`rlinf/models/embodiment/openpi_au/`](../../../rlinf/models/embodiment/openpi_au/)
- 入口脚本：复用 [`examples/sft/train_vla_sft_au.py`](../../sft/train_vla_sft_au.py)
- Norm stats 脚本：复用 [`examples/au/pi/compute_norm_stats_au.py`](../pi/compute_norm_stats_au.py)
- Docker 镜像：[`b/gcp/demo4/Dockerfile.aupi_dev`](../../../b/gcp/demo4/Dockerfile.aupi_dev)

## 与 `au/pi` 的区别

| | `au/pi`（裸金属） | `au/pidk`（Docker） |
|---|---|---|
| 运行环境 | 宿主机 Python venv | Docker 容器 (`rlinf-aupi-dev:260702`) |
| 节点数 | 固定单机 | 1-N 机（通过 `NUM_NODES` 配置） |
| 网络 | 直接使用宿主机 | `--network=host`（容器共享宿主机网络） |
| GPU 分配 | `component_placement: 0-7` | `component_placement: actor: all` |
| 代码/依赖 | 宿主机安装 | 镜像内 editable install + bind mount 源码 |

## 前置条件

1. **Docker 镜像** `rlinf-aupi-dev:260702`。构建方法（在 RLinf 仓库根目录执行）：
   ```bash
   docker buildx build \
     -f b/gcp/demo4/Dockerfile.aupi_dev \
     --build-context aupi05=/home/physical/SRC/Robot/aupi05 \
     -t rlinf-aupi-dev:260702 --load .
   ```
2. **NVIDIA Container Toolkit**（`nvidia-docker`），确保 `docker run --gpus all` 可用。
3. **pi0.5 基座权重**：默认 `/mnt/r/CKPT/VLA/pi05_base_pt_fp32`（可用 `BASE_CKPT=` 覆盖）。
4. **LeRobot 数据集**：默认 `/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1`（可用 `RLINF_PUSHDOOR_DATA=` 覆盖）。
5. **多机训练**：各节点间端口 6379（Ray）可达；可用 `RAY_PORT=` 换端口。

## 三步跑通

### 1. 计算归一化统计（norm_stats）

在 Docker 容器内单卡运行，结果写到 `_ckpt/pi05_pushdoor_tst1/rlinf/pushdoor_open0622/norm_stats.json`：

```bash
bash examples/au/pidk/run_norm_stats.sh
```

冒烟默认 `MAX_FRAMES=1024`；全量统计设 `MAX_FRAMES=13136`。

### 2. 启动 SFT 训练

#### 单机（默认，8 卡）

```bash
bash examples/au/pidk/run_train.sh
```

#### 多机（每台机器上分别执行）

```bash
# Node 0（head，假设 IP 为 192.168.1.100）:
RANK=0 NUM_NODES=2 bash examples/au/pidk/run_train.sh

# Node 1（worker）:
RANK=1 NUM_NODES=2 HEAD_HOST=192.168.1.100 bash examples/au/pidk/run_train.sh
```

3 节点同理，在第 3 台设 `RANK=2`。`HEAD_HOST` 只需在 worker 节点上指定。

### 3. 查看产物

```bash
ls examples/au/pidk/_out/pi05_pushdoor_tst1_dk/checkpoints/
```

## 关键配置一览（[`pushdoor_sft_pi05_au_dk.yaml`](pushdoor_sft_pi05_au_dk.yaml)）

- `cluster.num_nodes: ${RLINF_NUM_NODES}`（由 bootstrap.sh 自动设置）
- `cluster.component_placement: actor: all`（FSDP 跨所有节点所有 GPU）
- `actor.micro_batch_size: 1`，`global_batch_size: ${RLINF_GLOBAL_BATCH}`（自动计算 `NUM_NODES * 8`）
- `actor.model.action_dim: 23`，`openpi.config_name: pi05_pushdoor`
- 精度：openpi 自管 selective bf16，FSDP mixed_precision 全 null
- 学习率：`openpi_cosine`，warmup 100 / decay 1000 步，peak `5e-5` → `5e-6`；`ema_decay: 0.999`
- `runner.max_steps: 50`，`save_interval: 25`（冒烟；真训练请调大）

## Docker 运行细节

`run_train.sh` 调用 `docker run` 时使用以下关键参数：

- `--network=host`：容器共享宿主机网络栈，Ray 和 NCCL 可直接跨节点通信
- `--shm-size=64g`：PyTorch DataLoader 多 worker 需大 shared memory，避免 Bus error
- `--ulimit memlock=-1`：NCCL 需 pin GPU memory 做 RDMA
- `-v /mnt/r:/mnt/r:ro`：基座权重软链接指向 `/mnt/r/`，需在容器内可见
- bind mount 源码到 `/workspace/RLinf` 和 `/workspace/aupi05`：与镜像 editable install 路径一致

## 常见问题 / 可调项

- **改 GPU 数**：单机少于 8 卡时，在 `run_train.sh` 的 `docker run` 前加
  `--gpus '"device=0,1,2,3"'`（替换 `--gpus all`），并调整 `RLINF_GLOBAL_BATCH`。
- **改节点数**：设 `NUM_NODES=N`，`global_batch_size` 由 bootstrap.sh 自动算为 `N * 8`。
- **换镜像版本**：设 `IMAGE=rlinf-aupi-dev:<tag>`。
- **换基座权重**：设 `BASE_CKPT=/path/to/your_ckpt_dir`。
- **容器内调试**：
  ```bash
  docker run --rm -it --gpus all --network=host \
    -v /home/physical/SRC/RL/RLinf:/workspace/RLinf \
    -v /home/physical/SRC/Robot/aupi05:/workspace/aupi05 \
    rlinf-aupi-dev:260702 bash
  ```
- **Ray 端口冲突**：宿主机已有 Ray 集群占用 6379 时，设 `RAY_PORT=6380`。
- **NCCL 网卡选择**：多网卡环境下可设 `NCCL_SOCKET_IFNAME=eth0`（加到 `docker run -e` 中）。

## 目录内容

- `pushdoor_sft_pi05_au_dk.yaml` —— 训练配置（支持 1-N 节点）
- `bootstrap.sh` —— 容器内入口：Ray 集群组建 + 训练启动
- `run_train.sh` —— 宿主机启动脚本：权重 staging + 预检 + docker run
- `run_norm_stats.sh` —— 在 Docker 内计算 norm_stats
- `_ckpt/`、`_out/` —— 运行时生成（权重软链接 + norm_stats / checkpoints + 日志），不入库

---

## 踩坑记录（跑通过程中遇到的 error 及修复）

### Error 1: Docker 无法发现 GPU — `no known GPU vendor found`

**现象**：

```
docker: Error response from daemon: failed to discover GPU vendor from CDI: no known GPU vendor found
```

`docker run --gpus all` 直接报错退出（exit code 125），容器完全无法启动。

**根因**：

宿主机未安装 **NVIDIA Container Toolkit**（`nvidia-container-toolkit`）。Docker 的 `--gpus` 标志
依赖该工具包通过 CDI（Container Device Interface）或旧版 `nvidia-container-runtime` 向容器注入
GPU 设备和驱动库。虽然宿主机上 `/dev/nvidia*` 设备节点和驱动均正常（`nvidia-smi` 可用），但
Docker 运行时没有对应的 GPU vendor 插件，因此拒绝 `--gpus` 请求。

**修复方法**：

```bash
# 1. 添加 NVIDIA Container Toolkit APT 源
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

# 2. 安装
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# 3. 配置 Docker 运行时并重启
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4. 验证
docker run --rm --gpus all rlinf-aupi-dev:260702 nvidia-smi -L
```

### Error 2: `num_nodes` 类型错误 — `AssertionError: 'num_nodes' must be a positive integer`

**现象**：

```
File "/workspace/RLinf/rlinf/scheduler/cluster/config.py", line 594, in __post_init__
    assert type(self.num_nodes) is int and self.num_nodes > 0, (
AssertionError: 'num_nodes' must be a positive integer. But got 1 of type <class 'str'>.
```

训练脚本启动后，Hydra 加载配置阶段即报错。`num_nodes` 的值是字符串 `"1"` 而非整数 `1`。

**根因**：

YAML 配置中使用了 `${oc.env:RLINF_NUM_NODES,1}` 从环境变量读取 `num_nodes`。OmegaConf 的
`oc.env` resolver **始终返回字符串**（即使环境变量的值看起来是数字），而 RLinf 的
`ClusterConfig.__post_init__` 用 `type(self.num_nodes) is int` 做严格类型检查，字符串
`"1"` 无法通过。同理 `global_batch_size` 也有相同问题（虽然该字段的校验恰好更宽松没报错）。

**修复方法**：

1. 在 YAML 配置中使用字面整数作为默认值（不再用 `oc.env`）：
   ```yaml
   cluster:
     num_nodes: 1          # 不用 ${oc.env:RLINF_NUM_NODES,1}
   actor:
     global_batch_size: 8   # 不用 ${oc.env:RLINF_GLOBAL_BATCH,8}
   ```

2. 在 `bootstrap.sh` 中通过 Hydra 命令行 override 传入（Hydra CLI 会正确解析为 int）：
   ```bash
   python examples/sft/train_vla_sft_au.py \
     --config-path "${CONFIG_DIR}" \
     --config-name "${CONFIG_NAME}" \
     cluster.num_nodes="${NUM_NODES}" \
     actor.global_batch_size="${RLINF_GLOBAL_BATCH}"
   ```

   Hydra 的 CLI override 解析器会将 `cluster.num_nodes=1` 解析为整数 `1`，而非字符串 `"1"`。
