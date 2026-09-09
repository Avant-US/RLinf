# 4wvla realworld deploy
深入分析 @RLinf/ 代码库的真实代码, 以及RLinf里的各种md文档, 以及 @RLinf/docs/ 中的RLinf官方rst文档. 深入分析 @4WVLA/b/d/Frk/ 中各个落地实施方案中与数据集和训练相关的内容, 特别是 @4WVLA/b/d/Frk/plug_p2sft.md 和 @4WVLA/b/d/Frk/plug_p2sft_0907LOG.md 就是针对在"Franka 机器人做插插座任务"的数据上做微调训练的实施落地文档和微调训练时的日志. 请深入分析这些与4WVLA训练相关的文档, 以及深入分析 @4WVLA/ 中的实际代码, 特别是与"在 Franka 机器人做插插座任务的数据集上训练"相关的文档与代码. 参考 https://rlinf.readthedocs.io/en/latest/rst_source/extending/new_model_sft.html (对应 @RLinf/docs/source-en/rst_source/extending/new_model_sft.rst )以及它的相关代码, 写一份如何将 @4WVLA 模型整合添加到 @RLinf 中并加载相关checkpoint的设计与实施落地方案, 这份方案写在@RLinf/b/d/frk1/4wvla_rlinf_1.md 中. 然后, 参考 https://rlinf.readthedocs.io/en/latest/rst_source/evaluations/guides/realworld.html (对应 @RLinf/docs/source-en/rst_source/evaluations/guides/realworld.rst )以及它的相关代码, 写一篇真机评估的设计与实施落地方案. 方案写到 @RLinf/b/d/frk1/4wvla_rlinf_eval_1.md 中.

# 对于设计与实施落地方案有如下要求

对于 @RLinf/b/d/frk1/4wvla_rlinf_1.md 和 @RLinf/b/d/frk1/4wvla_rlinf_eval_1.md 这两篇文档, 我有如下要求:

- 方案要基于对目前服务器的软硬件环境的深入分析.(该服务器的软硬件环境包括了它连接的Franka机器人和docker上的RLinf相关的image)
- 代码或项目路径用本代码库的路径, 不要写死, 要能推理出来那种.
- 要尽量识别和抽象出会根据不同实验, 不同模型的SFT训练和eval评估的改变而改变的变量, 并抽取和罗列出来, 写出它们的含义, 作用, 实际生效的值, 以及具体在哪个文件的哪行取这个生效值的.
- 文档要自包含, 不要引用其它文档的内容, 但要标明内容的出处.  文档要包含详细的测试和验收部分.
- 方案要基于对现有代码的深入分析, 基于实际已有的代码. 同时要尽量复用已有的脚本或代码. 要遵守扩展大于修改的设计原则.
- 文档要把要增删改哪些文件或代码, 以及增删改哪些内容都要列出来, 要细到代码级别, 也要详细解释为什么要做这样的增删改, 也要列出复用了什么, 以及为什么要这样复用. 也要有专门章节描述, 当SFT训练或evla评估开始后, 各个脚本和代码间的调用顺序, 以及它们各自负责什么工作, 数据是如何流动的, 输入和输出分别涉及什么关键目录等等.
- 对于每个功能的增加,修改和删除都要有相应的测试用例与验收脚本, 以保证代码的正确与向前兼容.
- 文档要包括操作手册部分, 操作手册要详细到即使对该项目一无所知的第三方工程师按照该操作手册部分一步一步地执行也能成功解决问题完成SFT训练或eval评估. 除了一步一步的步骤也写清楚外, 也要写清楚做SFT训练或eval评估前, 用户要做什么? 要收集并提供什么信息? 要配置什么? 如何配置? 等等.
- 最重要的是, 对代码, 配置或其它文件所做的增删改等操作要兼容之前的功能, 不能因为加入一个功能而破坏了以前的功能.


请基于上述要求, 对文档 `4wvla_rlinf_1.md` 和 `4wvla_rlinf_eval_1.md` 进行改良.