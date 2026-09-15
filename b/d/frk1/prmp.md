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

---
按 @RLinf/b/d/frk1/4wvla_rlinf_2.md 进行实现, 并进行测试和验收. 过程中若遇到error就fix, 直到所有测试和验收都通过. 记录所有训练过程中的一切细节, 包括但不限于:所有的error及其根因分析, fix方案, 记录所有的操作, 命令, 关键路径和任何文件的增删改以及做这些操作的原因, 过程中的一切细节都记录在  @RLinf/b/d/frk1/4wvla_rlinf_2_0909LOG.md 后面. 

# 4wvla 推理2

参考 @RLinf/b/d/frk1/franka_3LOG.md , @RLinf/b/d/frk1/dmo_place_1LOG.md, @RLinf/b/d/frk1/dmo_place_1.md , @RLinf/b/d/frk1/dmo_place_2LOG.md , @RLinf/b/d/frk1/dmo_place_2.md , @RLinf/b/d/frk1/franka_3.md 中的真机RL方案和实践日志, 里面也有双容器部署与通过 franky 和 libfranka 去操控 Franka 机器人的内容.

参考 @4WVLA/b/d/Frk/dta_4dtrj_plan_0904LOG.md , @4WVLA/b/d/Frk/dta_4dtrj_plan.md 中处理训练数据的方案和实践日志. 参考 @4WVLA/b/d/Frk/plug_p1warmup_0907LOG.md , @4WVLA/b/d/Frk/plug_p1warmup.md , @4WVLA/b/d/Frk/plug_p2sft_0907LOG.md , @4WVLA/b/d/Frk/plug_p2sft.md 中的训练方案和实践日志. 我们要用到的checkpoint ( /home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/ ) 就是基于这些数据处理方案, 训练方案, 训练而得到的. 特别要注意这些方案中的 bbox 和 归一化 相关内容, 部署推理时必须与其一致.

对于 @RLinf/b/d/frk1/4wvla_rlinf_eval_2.md, 我有如下改良要求:
- 通过 RLinf 的插件或扩展模式进行功能的增加, 尽量不改 RLinf 原来的代码.
- 使用类似真机RL的双docker容器方案.
- 要使用 @RLinf/b/x/franky_ext/ 中的 env 和 controller, 通过 franky 和 libfranka 去操控 Franka 机器人, 不要通过ROS.
- 注意参考上面提到的数据处理方案, 训练方案以及相关日志等, 注意训推一致性, 特别要注意这些方案中的 bbox 和 归一化 相关内容, 部署推理时必须与其一致.
- 测试和验收方案要分成"需要连真机"的和"不需要连真机"的两类, 以便不连真机就可以做尽量完善的测试.

请根据我的要求, 对`4wvla_rlinf_eval_2.md`做改良.

# 改良 4wvla_rlinf_eval_3A2.md

我对 4wvla_rlinf_eval_3A2.md 有如下改良意见:

- 即使不用ray也要复用现在的rlinf的docker镜像或容器
- 尽量采用扩展的方式, 而不是修改 RLinf 原来代码的方式. 如果非要修改 RLinf 代码请详细说明理由. 
- 扩展的代码或脚本写在 RLmm/b/x/four_dwvla_ext/ 或 RLmm/b/x/franky_ext/ 中, 自己另外建一个目录也行, 但要详细说明理由.
- 把  @4WVLA/b/d/Frk/dta_4dtrj_plan_0904LOG.md 和 @4WVLA/b/d/Frk/dta_4dtrj_plan.md 提到的`bounding box`或`bbox`相关的问题也要考虑进去, 保证4D信息的一致性, 有效性, 以及数据归一化的一致性和有效性.
- 要考虑到 @RLmm/b/d/frk1/franka_3LOG.md 和 @RLmm/b/d/frk1/franka_3.md 中提到的 `safety box` 的相关问题.
- 参考 @RLmm/b/d/frk1/bx_analy_cp25.md , 仔细检查各种 box 的处理有没有问题, 七万不要出错, 或者弄乱了.
- 增加一个程序,可以让 Franka 机器人摆出 bbox 边缘上的, 或 safe box 边缘上的极限动作.
- 列出这个方案A"纯VLA评估"方案用了哪些 RLmm (也就是原 RLinf) 中关于 RLT 的代码或配置, 用在什么地方, 怎么用的, 有没有修改或扩展.
- 要对新增或修改的功能或代码设计可落地的详尽的测试与验收方案, 并且把测试和验收方案分成"不需要连接真机"和"需要连接真机"两类.

---
请基于我上述的意见, 对 @RLmm/b/d/frk1/4wvla_rlinf_eval_3A2.md 进行改良.

4wvla_rlinf_eval_3A2.md 是一个完整的自包含的方案文档. 不要只引用或提一下其它文档的内容然后让读者去看其它文档, 不要这样! 请把 @RLmm/b/d/frk1/4wvla_rlinf_eval_3A2.md 改成完整且自包含的文档.

# RLmm(RLinf)/RLT

参考RLT算法 ( 参考 RL Token: https://www.pi.website/research/rlt ) , https://rlinf.readthedocs.io/en/latest/rst_source/examples/embodied/rlt.html 和对应的 @RLmm/docs/source-zh/rst_source/examples/embodied/rlt.rst. 深入分析 @RLmm/ 里的代码和文档, 基于RLmm 中真实的代码和文档, 写一份关于 RLmm(也就是原生的RLinf)是如何实现 RLT 算法的分析报告. 你的同事以前写过一份关于 RLinf 如何实现 RLT 算法的分析报告在 @RLmm/b/d/rltx/rlt_code_analyz2.markdown 中, 但漏了不少细节, 而且有一些分析和解释没有说到本质上也没有说清楚是基于那些代码做的分析和解释, 那份老报告也没有一个从宏观到微观的按照操作和执行过程进行的逐渐分解下去的清晰易懂的分析与解析, 请检查一下你的同事写旧文档有些什么优点和缺点, 缺点弥补一下, 优点进一步发扬, 但你要写一篇更好的关于 RLinf 是如何实现 RLT 算法的分析文档. 该文档要有从宏观到微观的, 整个 rlt 操作和运行流程的UML图(包括但不限于静态的类图, 模块图, 等等, 以及动态的时序图,数据流图, 等等), 以及对这些图的解释, 必要时可以举例说明, 可以有分stage和分模块的总图, 然后有细化到子模块的代码级别的子图, 重点是每个代码或重点类之间是如何划分职责, 如何调用与协同的. 在进行动态方面的分析前, 最好先把静态方面的分析做好, 比如有些什么模块, 各负责什么, 有些什么特别的名词和变量, 各具体指什么意思, 这些模块和类之间的职责划分和协作关系也要从宏观到微观地画出来和解释清楚. 另外, 重要的代码逻辑也要按真实代码去解释. 请把你的深入分析和解析写到在 @RLmm/b/d/rltx/rlt_code_analyz3.markdown 中 .

# 真机 plug eval

请按照"用checkpoint 在 Franka 机器人上做纯VLA真机评估的实施落地方案(含详细操作手册)"   @RLmm/b/d/frk1/4wvla_rlinf_eval_3A3.md 中 里的指示 ,  生成和修改代码和脚本等, 然后进行`不需要连接真机的测试 (离线)`的测试和验收. 训练过程中若遇到error就fix, 直到所有测试和验收全都通过. 记录过程中的一切细节, 包括但不限于:所有的error及其根因分析, fix方案, 记录所有的操作, 命令, 关键路径和任何文件的增删改以及做这些操作的原因, 过程中的一切细节都记录在 @RLmm/b/d/frk1/4wvla_rlinf_eval_3A3_off0915LOG.md 后面. 


hf download --token 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 --cache-dir /home/nvidia/.cache/huggingface/hub "Qwen/Qwen3.5-2B"

参考RLT算法 ( 参考 RL Token: https://www.pi.website/research/rlt ) , https://rlinf.readthedocs.io/en/latest/rst_source/examples/embodied/rlt.html 和对应的 @RLmm/docs/source-zh/rst_source/examples/embodied/rlt.rst. 深入分析 @RLmm/ 里的代码和文档, 特别是关于RLT算法实现的代码和文档, 也可参考这份关于 RLmm(也就是原生的RLinf)是如何实现 RLT 算法的分析报告 @RLmm/b/d/rltx/rlt_code_analyz3.markdown. 同时深入分析 @4WVLA/ 的代码, 以及参考在 Franka 上用RLmm(也就是RLinf)对 4DWVLA(即4WVLA) 的模型checkpoint进行真机评估的实施方案和操作手册 @RLmm/b/d/frk1/4wvla_rlinf_eval_3A3.md . 深入思考一下如何用 RLmm(即Rlinf) 的 RLT 实现, 对4DWVLA(即4WVLA) 的模型checkpoint 进行 RLT 的 `Stage 1` 训练, 也就是对4DWVLA(即4WVLA) 的模型checkpoint  "VLA SFT + RLT token transformer" 的训练. 然后把对4DWVLA(即4WVLA)进行 RLT 的 `Stage 1` 训练的实施和落地方案写在 @RLmm/b/d/rltx/4dwvla_rlt1_1.markdown 中.