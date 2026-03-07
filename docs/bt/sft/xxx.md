# 资料
docs/source-zh/rst_source/examples/embodied/sft_openpi.rst
- 数据:pi0_libero 对应 rlinf/models/embodiment/openpi/dataconfig/__init__.py 中 name 为 `pi0_libero`的`TrainConfig`中的`repo_id="physical-intelligence/libero"`,也就是[pi版的libero](https://huggingface.co/datasets/physical-intelligence/libero)
- 参考文本是如何安装的
- 在docker镜像的py虚拟环境文件夹`/opt/venv/`中有多个虚拟环境,默认是openvla,要自己`source /opt/venv/openpi/bin/activate`到pi的虚拟环境
- 34.

# 调用
- examples/sft/run_vla_sft.sh 对应配置文件为 libero_sft_openpi.yaml 和 examples/sft/config/model/pi0.yaml
  - examples/sft/train_vla_sft.py
    - rlinf/runners/sft_runner.py
      - __init__()
      - init_workers()
        - rlinf/workers/sft/fsdp_vla_sft_worker.py 有FSDPVlaSftWorker及其父类的很多方法
      - run()

- FSDPVlaSftWorker.build_dataloader()
  - 在 rlinf/workers/sft/fsdp_vla_sft_worker.py 可看到`from rlinf.models.embodiment.openpi.dataconfig import get_openpi_config`
    - rlinf/models/embodiment/openpi/dataconfig/__init__.py 的 get_openpi_config 其实就是基于`TrainConfig`的name从 _CONFIGS_DICT 即 _CONFIGS 中拿对应的使用`openai`包所需的TrainConfig