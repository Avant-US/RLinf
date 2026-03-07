# 资料
docs/source-zh/rst_source/examples/embodied/isaaclab.rst
- 数据:pi0_libero 对应 rlinf/models/embodiment/openpi/dataconfig/__init__.py 中 name 为 `pi0_libero`的`TrainConfig`中的`repo_id="physical-intelligence/libero"`,也就是[pi版的libero](https://huggingface.co/datasets/physical-intelligence/libero)
- 参考文本是如何安装的
- 在docker镜像的py虚拟环境文件夹`/opt/venv/`中有多个虚拟环境,默认是openvla,要自己`source /opt/venv/openpi/bin/activate`到pi的虚拟环境
- 关注如何在isaaclab中添加自定义任务那一节,因为例子用的是自定义任务
- docker中用到的虚拟环境叫gr00t而不是isaaclab
- 边在仿真环境中训练还可以边做[数据采集](../../../docs/source-zh/rst_source/tutorials/components/data_collection.rst)哦

## 自定义IsaacLab任务
- docs/source-zh/rst_source/examples/embodied/isaaclab.rst 中自定义任务一节
- https://github.com/isaac-sim/IsaacLab/compare/main...RLinf:IsaacLab:main 可看到RLInf版的IsaacLab改了些什么,其实就是加了个自定义任务,这里可看到加任务时需要改IsaacLab的什么文件
- 除了改IsaacLab的文件,也要在RLinf中改文件,参考 rlinf/envs/isaaclab/__init__.py 中的代码

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
- `bash examples/embodiment/run_embodiment.sh isaaclab_franka_stack_cube_ppo_gr00t` > train_embodied_agent.py
- runer.init_workers 各个wrk的初始化 env_wrk.py,hf_wrk.py,fsdp_act_wrk的EmbodiedFSDPActor
  - env_wrk>init_wrk>envs/init.py.get_env_cls 根据cfg配置的task或env获取对应的env实现类
    - IsaacLab 对应了 envs/isaaclab/init.py 这个例子的env实现类是`IsaaclabStackCubeEnv(IsaaclabBaseEnv)`
- runer.run
  - envWrk.interact()
    - `env_outputs = self.bootstrap_step()`
      - bootstrap_step>IsaaclabStackCubeEnv(IsaaclabBaseEnv).reset>IsaaclabStackCubeEnv._make_env_function
