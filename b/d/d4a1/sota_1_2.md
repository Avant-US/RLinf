# chosenls_1：2026 SOTA 深度重查

## 口径与结论先行

- **检索截止日**：2026-07-18。
- **2026 新方法**：论文首发、模型发布，或正式会议录发生在 2026。若预印本首发于 2025、但正式录用于 ICLR/ICRA/AAAI/CVPR 2026，本文会同时写明两种日期。
- **2026 新榜单结果（旧模型）**：只表示该旧模型在 2026 年榜单/论文表中仍领先，**不冒充 2026 新方法**。
- **官方榜**：仅限 benchmark 官方网页、官方 HF/CodaBench/EvalAI 榜、挑战官网或 benchmark 论文表。
- **间接对比**：2026 方法在同一数据/协议上由论文自行测试，或只在高度相邻任务上达到 SOTA；后者明确写“不可直接混榜”。
- **数量不足时不补旧模型**：很多条目没有持续 leaderboard，或 2026 年只有一项可核实新方法。本文不为了凑 Top 3 把 2024/2025 工作包装成 2026 SOTA。
- **重要更正**：VBench-2.0、WorldScore、Physics-IQ、PhyGenBench 的总分口径互不等价；UniOcc、Cam4DOcc、Occ3D 的 occupancy 定义也不能混排。

## 2026 真正新增结果总览

- 有可验证 2026 官方榜结果：**WorldScore、Physics-IQ、RoboWM-Bench、GigaBrain Challenge、AgiBot World Challenge、HOT3D BOP model-free（X0）**。
- 有 2026 同协议论文新高分但未写回官方榜：**PhyGenBench（CoECT 0.66）、Cam4DOcc（IR-WM）、Ego-Exo4D Hand Forecasting（EggHand）**；STM4D 已撤稿，不计正式 2026 方法。
- 仅有 2026 相邻任务新方法、没有目标榜直接分数：**4DWorldBench、DynamicVerse、UniOcc、EgoDex，以及 HOT3D HANDS 的自建 split 论文**。
- 截止检索日找不到严格 2026 新榜结果：**WorldModelBench 官方榜、VBench-2.0 官方榜 Top 3、Ego-Exo4D 2026 Ego-Pose Body 最终公开分数**；2026 Ego-Exo4D 根本没有 Hand Pose/3D Hand Forecasting challenge。

---

## 1. 4DWorldBench

- **官方入口**：[项目页](https://yeppp27.github.io/4DWorldBench.github.io/) · [arXiv:2511.19836](https://arxiv.org/abs/2511.19836) · [CVPR 2026 正式论文](https://openaccess.thecvf.com/content/CVPR2026/html/Lu_4DWorldBench_A_Comprehensive_Evaluation_Framework_for_3D4D_World_Generation_Models_CVPR_2026_paper.html)
- **官方协议**：Image-to-4D、Video-to-4D、Text-to-4D；评估感知质量、条件对齐、物理真实性和 4D 一致性。

### 2026 官方 Top 3

**没有严格意义上的“2026 新方法 Top 3”。** CVPR 2026 benchmark 论文表中的领先生成器均在 2025 或更早首发，而且官网没有持续开放提交榜。

作为“2026 论文表中的旧模型”留档：

1. **Diffusion as Shader（2026 新榜单结果，旧模型）**：Image-to-4D Overall **0.763**，该子表第 1；首发 arXiv 2025。参考图与 3D tracking video 控制扩散视频，约用不足 10K 视频微调。[arXiv:2501.03847](https://arxiv.org/abs/2501.03847) · [项目页](https://igl-hkust.github.io/das/) · [GitHub](https://github.com/igl-hkust/diffusionasshader)；代码/权重公开。
2. **ReCamMaster（2026 新榜单结果，旧模型）**：Video-to-4D Overall **0.685**，该子表第 1；使用约 136K 多相机视频训练相机轨迹重定向。[arXiv:2503.11647](https://arxiv.org/abs/2503.11647) · [项目页](https://jianhongbai.github.io/ReCamMaster/) · [GitHub](https://github.com/KwaiVGI/ReCamMaster)；代码公开，完整效果依赖骨干。
3. **TrajectoryCrafter（2026 新榜单结果，旧模型）**：Video-to-4D Overall **0.670**，该子表第 2；双流扩散进行单目视频新视角与轨迹编辑。[arXiv:2503.05638](https://arxiv.org/abs/2503.05638) · [项目页](https://trajectorycrafter.github.io/) · [GitHub](https://github.com/TrajectoryCrafter/TrajectoryCrafter)；部分开源。

### 2026 间接 Top 3

#### 1) NeoVerse（2026 间接对比）

- **年份证据**：2026-01 首发，CVPR 2026 Highlight。
- **方法**：pose-free 前馈 4DGS 重建器 + Wan2.1 视频扩散；在线从稀疏关键帧重建 4D，再用单目退化模拟训练生成器。
- **数据/训练**：可扩展至约 1M 段 in-the-wild 单目视频；无需每场景优化。
- **表现**：论文在自身重建/生成协议上声明 SOTA；**未在 4DWorldBench 官方表提交，不能与 0.763/0.685 直接排名**。
- **链接/开源**：[arXiv:2601.00393](https://arxiv.org/abs/2601.00393) · [项目页](https://neoverse-4d.github.io/) · [GitHub](https://github.com/IamCreateAI/NeoVerse) · [权重](https://huggingface.co/Yuppie1204/NeoVerse)；推理、checkpoint 已公开。

#### 2) VerseCrafter（2026 间接对比）

- **年份证据**：2026-01 首发，CVPR 2026。
- **方法**：以背景点云和逐物体 3D Gaussian trajectory 表示 4D 几何控制，渲染成控制图，经 GeoAdapter 注入冻结 Wan2.1。
- **数据/训练**：VerseControl4D，约 35K 样本；从 Sekai-Real-HQ/SpatialVID-HQ 自动估计相机、深度、mask 和物体轨迹。
- **表现**：相机与多物体运动控制优于论文基线；**未报告 4DWorldBench Overall**。
- **链接/开源**：[arXiv:2601.05138](https://arxiv.org/abs/2601.05138) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Zheng_VerseCrafter_Dynamic_Realistic_Video_World_Model_with_4D_Geometric_Control_CVPR_2026_paper.html) · [项目页](https://sixiaozheng.github.io/VerseCrafter_page/) · [GitHub](https://github.com/TencentARC/VerseCrafter) · [权重](https://huggingface.co/TencentARC/VerseCrafter)；推理与权重公开。

#### 3) Phys4D（2026 间接对比）

- **年份证据**：arXiv 2603.03485。
- **方法**：用 simulation-derived depth/motion/mask 将物理结构注入预训练视频扩散；伪监督预训练、物理 SFT、simulation-grounded RL 三阶段提升几何和长时物理一致性。
- **表现**：在作者自建 4D consistency diagnostics 上优于 appearance-driven 基线；**没有 4DWorldBench 分数**。
- **链接/开源**：[arXiv:2603.03485](https://arxiv.org/abs/2603.03485) · [项目页](https://sensational-brioche-7657e7.netlify.app/)；截至检索日页面写明 **Code coming soon**，无公开 GitHub。

---

## 2. WorldScore

- **官方入口**：[项目页](https://haoyi-duan.github.io/WorldScore/) · [GitHub](https://github.com/haoyi-duan/WorldScore) · [HF 官方榜](https://huggingface.co/spaces/Howieeeee/WorldScore_Leaderboard) · [官方 CSV 快照](https://huggingface.co/api/resolve-cache/spaces/Howieeeee/WorldScore_Leaderboard/8e76583bda77962c56d0dde32eda190d10c55154/leaderboard.csv?download=true)
- **协议**：Static 聚合 controllability + quality；Dynamic 再加入 motion accuracy/magnitude/smoothness。

### 2026 官方 Top 3

#### 1) EvoPhys-World（2026 官方榜）

- **年份证据**：官方 CSV 加入日期 **2026-06-03**。
- **表现**：Static **83.45**、Dynamic **73.78**，截至检索快照双榜第 1；Camera Control 94.56、Object Control 85.52、Content Alignment 83.37。
- **方法/训练**：项目页称其为 human-centric “5D” world model，由共享状态记忆、World Engine、World Policy 和 action-conditioned multiple-future rollout 构成；论文、训练配置和数据清单未公开，不能验证更多架构细节。
- **链接/开源**：[项目页](https://evophys.com/) · [WorldScore 官方榜](https://huggingface.co/spaces/Howieeeee/WorldScore_Leaderboard) · [官方 CSV](https://huggingface.co/api/resolve-cache/spaces/Howieeeee/WorldScore_Leaderboard/8e76583bda77962c56d0dde32eda190d10c55154/leaderboard.csv?download=true)；截至检索日 **无 arXiv、GitHub、权重或训练数据**，实际不可复现。

#### 2) WorldScape-0.2（2026 官方榜）

- **年份证据**：官方榜加入 **2026-05-24**；0.1 技术报告日期 2026-04-03。
- **表现**：Static **81.37**、Dynamic **73.62**，双榜第 2；Object Control 87.34、Motion Smoothness 81.51。
- **方法**：0.1 报告披露其为统一导航/操作动作条件的自回归视频扩散，加入 3DGS 派生 depth/render supervision 与几何记忆 KV cache；**0.2 相对 0.1 的完整技术改动没有公开论文说明**。
- **链接/开源**：[0.1 技术报告 PDF](https://manifoldai.cn/assets/file/WorldScape.pdf) · [Manifold AI 介绍](https://manifoldai.cn/blogs/WorldScape.html) · [官方榜](https://huggingface.co/spaces/Howieeeee/WorldScore_Leaderboard)；`worldscape.io` 截至审计时返回 404，未找到 0.2 的独立 arXiv、GitHub 或权重，实际不可复现。

#### 3) EonWorld（2026 官方榜）

- **年份证据**：官方 CSV 加入日期 **2026-04-30**。
- **表现**：Static **81.08**、Dynamic **73.37**，双榜第 3；3D Consistency 92.76、Style 96.70。
- **方法/训练**：X-Era 将其描述为面向机器人的 VWA 世界动作模型；架构、训练数据和复现实验未公开，不能从宣传材料反推技术细节。
- **链接/开源**：[WorldScore 官方榜](https://huggingface.co/spaces/Howieeeee/WorldScore_Leaderboard) · [X-Era GitHub 组织](https://github.com/X-EraAI) · [公司页](https://www.x-era.com/)；官方 CSV 标为 Open Source，但截至检索日 **没有 EonWorld 论文、代码仓或权重**，故实际按未开源处理。

### 2026 间接 Top 3（与官方前三去重后）

1. **FantasyWorld-1.0（2026 官方与间接对比，当前第 5）**：ICLR 2026；冻结视频 foundation model，加可训练几何分支，让 video latent 与 implicit 3D field 双向监督。官方榜 Static **80.45** / Dynamic **71.39**，已被更新模型超越，但仍是完全开源的 2026 强方法。[arXiv:2509.21657](https://arxiv.org/abs/2509.21657) · [ICLR 2026 OpenReview](https://openreview.net/forum?id=3q9vHEqsNx) · [项目页](https://fantasy-amap.github.io/fantasy-world/) · [GitHub](https://github.com/Fantasy-AMAP/fantasy-world)。
2. **Worldscape-MoE（2026 间接对比）**：共享 world-dynamics backbone + camera/robot/hand modality experts，渐进 MoE tuning；尚无 WorldScore 同协议分数。[arXiv:2607.03964](https://arxiv.org/abs/2607.03964) · [项目页](https://worldscape-moe.com/) · [GitHub](https://github.com/EmbodiedCity/Worldscape-MoE.code) · [HF 权重页](https://huggingface.co/EmbodiedCity/Worldscape-MoE)；仓库称 7 月底前完成发布，当前不算完整开源。
3. **NeoVerse（2026 间接对比）**：pose-free 前馈 4DGS 重建器 + Wan2.1 视频扩散，可扩展至约 1M 段单目视频；未上 WorldScore 官方榜。[arXiv:2601.00393](https://arxiv.org/abs/2601.00393) · [项目页](https://neoverse-4d.github.io/) · [GitHub](https://github.com/IamCreateAI/NeoVerse) · [权重](https://huggingface.co/Yuppie1204/NeoVerse)。

> TeleWorld 报告 Static 78.23 / Dynamic 66.73，但 arXiv v1 实际在 **2025-12-31** 发布，且没有公开代码，因此列入文末“旧模型/边界案例”，不占 2026 新方法名额。[arXiv:2601.00051](https://arxiv.org/abs/2601.00051)

---

## 3. DynamicVerse

- **入口**：[项目页](https://dynamic-verse.github.io/) · [arXiv:2512.03000](https://arxiv.org/abs/2512.03000) · [GitHub](https://github.com/Dynamics-X/DynamicVerse) · [HF 数据](https://huggingface.co/datasets/kairunwen/DynamicVerse)
- **性质**：100K+ 4D scenes、800K masklets、10M frames 的数据与自动标注管线；不是生成 leaderboard。

### 2026 官方 Top 3

**不存在。** DynamicVerse 官方只给 DynamicGen 在 Sintel/KITTI/TUM 上的 depth/pose/intrinsics 验证；截至检索日没有“使用 DynamicVerse 训练后统一提交”的榜单。DynamicGen 预印本首发 2025-12，不计 2026 新方法。

### 2026 间接 Top 3

1. **NeoVerse（2026 间接对比）**：使用大规模 in-the-wild 单目视频扩展 4D world model，与 DynamicVerse 的数据扩展目标最接近；但未声明用 DynamicVerse 训练。[arXiv](https://arxiv.org/abs/2601.00393) · [GitHub](https://github.com/IamCreateAI/NeoVerse) · [权重](https://huggingface.co/Yuppie1204/NeoVerse)。
2. **VerseCrafter（2026 间接对比）**：自建 VerseControl4D 35K 控制数据；输出 camera + multi-object 4D control video，与 DynamicVerse 标注模态相邻，但不同数据协议。[arXiv](https://arxiv.org/abs/2601.05138) · [GitHub](https://github.com/TencentARC/VerseCrafter) · [权重](https://huggingface.co/TencentARC/VerseCrafter)。
3. **Phys4D（2026 间接对比）**：simulation-grounded RGB-D-motion 训练，直接优化几何/运动/物理 4D 一致性；未用 DynamicVerse 官方几何表。[arXiv](https://arxiv.org/abs/2603.03485) · [项目页](https://sensational-brioche-7657e7.netlify.app/)；代码未发布。

**结论**：DynamicVerse 目前更适合作为预训练/数据工程资源，而不是可给出“2026 模型 Top 3”的 benchmark。

---

## 4. WorldModelBench

- **入口**：[项目页](https://worldmodelbench-team.github.io/) · [GitHub](https://github.com/WorldModelBench-Team/WorldModelBench) · [arXiv:2502.20694](https://arxiv.org/abs/2502.20694) · [OpenReview](https://openreview.net/forum?id=a3hafrDzuA)
- **协议**：Instruction Following、Physics Adherence、Common Sense 聚合成 0–10 Total。

### 2026 官方 Top 3

**无可验证的 2026 新方法 Top 3。** 官方页面/论文新增表仍以 Veo 3（9.18）、Kling（9.10）、Wan2.1-T2V（9.04）为领先项；这些模型均不是 2026 首发方法，不能改称“2026 SOTA 模型”。

### 2026 间接 Top 3

1. **PHANTOM（2026 间接对比）**：Wan2.2-TI2V 视觉分支旁加入 V-JEPA2 latent-physics 分支，双向 cross-attention 联合预测视觉与物理状态；只用约 400K 训练视频。它在 Physics-IQ、VideoPhy/2、VBench-2.0 改善物理维，**未报告 WorldModelBench Total**。[arXiv:2604.08503](https://arxiv.org/abs/2604.08503) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Shen_PHANTOM_Physics-Infused_Video_Generation_via_Joint_Modeling_of_Visual_and_CVPR_2026_paper.html) · [项目页](https://plan-lab.github.io/projects/phantom/)；代码仍为 coming soon。
2. **NEWTON（2026 间接对比）**：Planner–Executor–Verifier 多轮工具规划；Flow-GRPO 只训练 planner，不改冻结视频生成器。VideoPhy-2 上 LTX joint 21.4→29.7、Veo3.1 30.7→37.4；**未测 WorldModelBench**。[arXiv:2605.18396](https://arxiv.org/abs/2605.18396) · [项目页](https://Newton026.github.io/newton) · [GitHub](https://github.com/CUTEPKQ/NEWTON)。
3. **PhysRAG（2026 间接对比）**：从 WISA-80K 筛出 7K 高质量物理视频，建检索库，以 learnable queries 将参考物理 dynamics 注入 Wan2.2-TI2V。PhyGenBench Avg 0.58；**未测 WorldModelBench**。[arXiv:2606.26916](https://arxiv.org/abs/2606.26916) · [GitHub](https://github.com/sediment1024/PhysRAG) · [模型](https://huggingface.co/sediment1024/PhysRAG) · [数据](https://huggingface.co/datasets/sediment1024/PhysRAG)；代码/模型/数据公开。

**结论**：2026 有更强的物理视频方法，但 WorldModelBench 官方榜没有同步评测，故不能写新的官方前三。

---

## 5. Physics-IQ

- **入口**：[项目页](https://physics-iq.github.io/) · [官方 GitHub/Verified 榜](https://github.com/google-deepmind/physics-IQ-benchmark) · [原论文](https://arxiv.org/abs/2501.09038)
- **年份说明**：benchmark 正式录用于 WACV 2026；榜单必须分 **i2v** 与 **v2v**，Verified 与 Original 也不可混排。

### 2026 官方 Top 3

#### 1) MAGI-1 24B + GeoPhys Best-of-16（2026 官方与间接对比）

- **年份证据**：GeoPhys arXiv 2026-06；MAGI-1 本体为 2025。
- **方法**：冻结 DINO/图像编码器，把逐帧 embedding 看作轨迹，以速度变化、曲率、角一致性、加速度和线性预测残差构造无训练 verifier。
- **表现**：Verified v2v **58.2±1.8**，榜首；Original 报 **64.5%**。相对 MAGI baseline 约 50.0 显著提升。
- **成本**：无需视频预训练/物理监督；项目报告相对 V-JEPA2 verifier 更快且显存更低。
- **链接/开源**：[GeoPhys arXiv:2606.20707](https://arxiv.org/abs/2606.20707) · [项目页](https://christianinterno.github.io/GeoPhys/) · [GitHub](https://github.com/ChristianInterno/GeoPhys)；MAGI [arXiv:2505.13211](https://arxiv.org/abs/2505.13211) · [GitHub](https://github.com/SandAI-org/MAGI-1)；完全开源。

#### 2) Cosmos3-Super-Image2Video（2026 官方榜）

- **表现**：Verified i2v **39.5±0.8**，i2v 第 1；不能与 v2v 58.2 当作同一输入设定。
- **方法/训练**：NVIDIA Cosmos3 世界基础模型的 I2V Super 变体；完整 Super 训练细节和权重未完全公开。
- **链接/开源**：[Cosmos3 技术报告 PDF](https://research.nvidia.com/labs/cosmos-lab/cosmos3/technical-report.pdf) · [开放系列仓 cosmos-predict2.5](https://github.com/nvidia-cosmos/cosmos-predict2.5)；**部分开源**，不能用 2.5 仓代替 3-Super 权重。

#### 3) Grok Imagine Video（2026 官方榜）

- **表现**：Verified i2v **34.8**，i2v 第 2（官方榜记录 2026-06-17）。
- **方法/训练**：xAI 闭源产品，论文、训练数据、checkpoint 均未公开。
- **链接/开源**：[xAI 产品入口](https://x.ai/) · [Physics-IQ 官方榜](https://github.com/google-deepmind/physics-IQ-benchmark)；未开源。

### 2026 间接 Top 3（去重后）

1. **WMReward（2026 官方与间接对比）**：用 V-JEPA2 surprise 作为奖励做 Best-of-N/去噪引导；Original Physics-IQ 中 MAGI-1+WMReward v2v **62.64%**，论文称 ICCV 2025 Challenge 第 1，但方法论文首发 2026。[arXiv:2601.10553](https://arxiv.org/abs/2601.10553) · [GitHub](https://github.com/facebookresearch/WMReward)；完全开源。
2. **PHANTOM（2026 间接对比）**：Physics-IQ single **29.59**、multi **27.53**；相对 Wan2.2-TI2V single 22.10 提升 33.9%。这是论文自测，不应覆盖 Verified 官方榜。[arXiv](https://arxiv.org/abs/2604.08503) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Shen_PHANTOM_Physics-Infused_Video_Generation_via_Joint_Modeling_of_Visual_and_CVPR_2026_paper.html) · [项目页](https://plan-lab.github.io/projects/phantom/)；代码未公开。

第三个严格 2026、且直接报告 Physics-IQ 的独立新方法未找到；不以 2025 MAGI 单模或旧 Wan/Hunyuan 补位。

---

## 6. VBench-2.0 / VBench++

- **入口**：[VBench-2.0 项目页](https://vchitect.github.io/VBench-2.0-project/) · [GitHub](https://github.com/Vchitect/VBench/tree/master/VBench-2.0) · [HF 榜](https://huggingface.co/spaces/Vchitect/VBench_Leaderboard) · [arXiv:2503.21755](https://arxiv.org/abs/2503.21755)
- **协议**：VBench-2.0 Total 是 Creativity、Commonsense、Controllability、Human Fidelity、Physics 五类均值；VBench++、原 VBench、VBench-2.0 不可混算。

### 2026 官方 Top 3

截至检索日，HF 官方榜能看到 2026 年页面更新，但没有足够版本/发布日期证据将前三全部认定为“2026 新方法”。旧稿中的 Veo 3、Vidu Q1、Wan2.1 不能再写作 2026 新模型。

### 2026 间接结果

#### PHANTOM（2026 间接对比，同 VBench-2.0 协议）

- **表现**：Total **51.84**；Creativity 45.51、Commonsense 61.43、Controllability 20.23、Human Fidelity 88.39、Physics **43.61**。基座 Wan2.2-TI2V-5B 为 Total 51.57 / Physics 40.19。
- **判定**：论文自行按官方代码测量，证明物理维提升，但 **51.84 并非官方实时榜 Top 3**。
- **方法/训练**：V-JEPA2 物理 latent 与视频 latent 双分支联合建模；约 400K 视频；不依赖显式 simulator。
- **链接/开源**：[arXiv:2604.08503](https://arxiv.org/abs/2604.08503) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Shen_PHANTOM_Physics-Infused_Video_Generation_via_Joint_Modeling_of_Visual_and_CVPR_2026_paper.html) · [项目页](https://plan-lab.github.io/projects/phantom/)；代码未公开。

PhysRAG 论文写“VBench”而不是可核实的 VBench-2.0 五类表；Seedance 2.0、Kling 3.0、Wan 2.7 虽是 2026 强视频模型，但未找到官方 VBench-2.0 分数。因此不伪造其 Total 或排名。

---

## 7. PhyGenBench

- **入口**：[项目页](https://phygenbench123.github.io/) · [GitHub](https://github.com/OpenGVLab/PhyGenBench) · [arXiv:2410.05363](https://arxiv.org/abs/2410.05363) · [ICML 2025](https://proceedings.mlr.press/v267/meng25c.html)
- **协议**：160 prompts、27 laws、Mechanics/Optics/Thermal/Material；PhyGenEval PCA 0–1。

### 2026 官方 Top 3

官方仓 leaderboard 截止检索日仍是旧表（Gen-3 0.51、Kling 0.49 等），没有把 2026 论文结果写回，因此 **无 2026 官方 Top 3**。

### 2026 同协议间接 Top 3

#### 1) CoECT：Chain of Event-Centric Causal Thought（2026 间接对比）

- **年份证据**：arXiv 2026-03；CVPR 2026。
- **方法**：Physics-driven Event Chain Reasoning 将现象拆成因果事件，以公式约束数值关系；Transition-aware Cross-modal Prompting 生成连贯叙述和关键帧，再驱动 CogVideoX-5B。
- **训练/数据**：生成器使用现成 CogVideoX-5B；GPT-OSS-20B 负责语言推理，Qwen-Image 生成关键帧；主要是推理/条件构造而非重训大视频骨干。
- **表现**：Mechanics **0.67**、Optics **0.72**、Thermal **0.65**、Material **0.60**、Avg **0.66**，论文表第 1；超过 PhysHPO 0.61。
- **链接/开源**：[arXiv:2603.09094](https://arxiv.org/abs/2603.09094) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Wang_Chain_of_Event-Centric_Causal_Thought_for_Physically_Plausible_Video_Generation_CVPR_2026_paper.html) · [GitHub](https://github.com/ZixuanWang0525/CoECT)；代码公开。

#### 2) PhysRAG（2026 间接对比）

- **年份证据**：arXiv 2026-06；仓库标注 ECCV 2026。
- **方法/训练**：WISA-80K 两阶段过滤得 7K 视频；检索相似物理动态，以 learnable query 注入 Wan2.2-TI2V-5B。
- **表现**：PhyGenBench Avg **0.58**，论文称其表中 SOTA；低于更早发布的 CoECT 0.66，因此在本次跨论文重排中为第 2。
- **链接/开源**：[arXiv:2606.26916](https://arxiv.org/abs/2606.26916) · [GitHub](https://github.com/sediment1024/PhysRAG) · [HF 模型](https://huggingface.co/sediment1024/PhysRAG) · [HF 数据](https://huggingface.co/datasets/sediment1024/PhysRAG)；完整度高。

#### 3) NEWTON（2026 间接对比）

- **年份证据**：arXiv 2026-05。
- **方法**：学习型 planner 在科学计算、prompt refinement、keyframe generation、video generator、verifier 之间多轮规划；Flow-GRPO on-policy 训练 planner。
- **表现**：在 PhyGenBench 将 LTX-Video-2B Avg **0.510→0.560**；Mechanics 0.500、Optics 0.647、Thermal 0.522、Material 0.542。它超过论文所列 Wan2.2 0.544，但低于 CoECT/PhysRAG。
- **链接/开源**：[arXiv:2605.18396](https://arxiv.org/abs/2605.18396) · [项目页](https://Newton026.github.io/newton) · [GitHub](https://github.com/CUTEPKQ/NEWTON)；代码公开。

---

## 9. UniOcc

- **入口**：[项目页](https://uniocc.github.io/) · [GitHub](https://github.com/tasl-lab/UniOcc) · [arXiv:2503.24381](https://arxiv.org/abs/2503.24381) · [ICCV 2025](https://openaccess.thecvf.com/content/ICCV2025/html/Wang_UniOcc_A_Unified_Benchmark_for_Occupancy_Forecasting_and_Prediction_in_ICCV_2025_paper.html)
- **协议限制**：UniOcc 同时包含 occupancy forecasting、camera occupancy prediction、cooperative occupancy；不同任务不能组成一个总榜。

### 2026 官方 Top 3

**无。** 官方仓和论文表没有 2026 新结果，也无持续提交 leaderboard。旧表中的 OccWorld+Voxel Flow、CVTOcc、CoHFF 都不是 2026 方法。

### 2026 间接结果

1. **IR-WM（2026 正式会议；间接对比）**：预印本 2025-10，正式 ICRA 2026。以前一时刻 BEV 为先验，只预测动作条件 residual，并用 alignment 抑制 rollout 误差。nuScenes inflated GMO：IoU_c **40.80**、IoU_f(2s) **37.20**、加权 IoU_f **38.20**；这是 Cam4DOcc-compatible 表，**未写入 UniOcc 官方多任务表**。[arXiv:2510.16729](https://arxiv.org/abs/2510.16729) · [GitHub/ir-wm](https://github.com/yuyang-cloud/Drive-OccWorld/tree/ir-wm) · [HF 权重](https://huggingface.co/Jianbiao/IR-WM) · [作者页 ICRA 2026 记录](https://jianbiaomei.github.io/)；代码和权重公开。

**STM4D 边界案例**：2025-09 投稿、2026-03 撤稿，OpenReview 状态为 **ICLR 2026 Conference Withdrawn Submission**；联合 3D volumetric temporal、2D segmentation forecasting 和 2D–3D interaction，Occ3D-nuScenes Avg mIoU **9.83**。它既未复现 UniOcc 官方表，也不是 2026 正式论文，不占 2026 方法名额。[OpenReview](https://openreview.net/forum?id=vfDJiI0dbu)。

没有第二、第三个可验证的 2026 方法直接复现 UniOcc 协议；不以 2025 PreWorld/OccProphet 补位。

---

## 10. Cam4DOcc

- **入口**：[CVPR 2024](https://openaccess.thecvf.com/content/CVPR2024/html/Ma_Cam4DOcc_Benchmark_for_Camera-Only_4D_Occupancy_Forecasting_in_Autonomous_Driving_CVPR_2024_paper.html) · [arXiv:2311.17663](https://arxiv.org/abs/2311.17663) · [GitHub](https://github.com/haomo-ai/Cam4DOcc)
- **主协议**：nuScenes，3 observed + 4 future frames；Inflated GMO 关注 IoU_c、IoU_f(2s)、时间加权 IoU_f。

### 2026 官方 Top 3

官方仓没有持续榜，因此不存在 2026 官方提交 Top 3。

### 2026 论文中的同协议 Top 3

#### 1) IR-WM（2026 正式会议；间接对比）

- **年份证据**：2025 首发，ICRA 2026 正式录用。
- **方法**：视觉 BEV current-state encoder + action-conditioned implicit residual world model + alignment calibration；同时连接规划头。
- **表现**：nuScenes inflated GMO **40.80 / 37.20 / 38.20**（IoU_c / IoU_f / 加权 IoU_f），高于 Drive-OccWorld 39.80/36.30/37.40；nuScenes-Occupancy 为 16.20/14.50/15.00。
- **链接/开源**：[arXiv:2510.16729](https://arxiv.org/abs/2510.16729) · [代码分支](https://github.com/yuyang-cloud/Drive-OccWorld/tree/ir-wm) · [HF 权重](https://huggingface.co/Jianbiao/IR-WM) · [ICRA 2026 记录](https://jianbiaomei.github.io/)；Apache-2.0，代码和权重公开。

#### 2) Drive-OccWorld（2026 新论文对比表中的旧方法）

- **年份/表现**：2024 arXiv、AAAI 2025；IR-WM Table I 中 **39.80 / 36.30 / 37.40**，是同一论文快照第 2，但不是 2026 新方法。
- **链接/开源**：[AAAI 2025](https://ojs.aaai.org/index.php/AAAI/article/view/33010) · [项目页](https://drive-occworld.github.io/) · [GitHub](https://github.com/yuyang-cloud/Drive-OccWorld)。

#### 3) OccProphet（2026 新论文对比表中的旧方法）

- **年份/表现**：ICLR 2025；IR-WM Table I 中 **34.36 / 26.94 / 29.15**，同一论文快照第 3，不是 2026 新方法。
- **链接/开源**：[ICLR 2025 OpenReview](https://openreview.net/forum?id=vC7AlY1ytz) · [项目页](https://jlchen-c.github.io/OccProphet/) · [GitHub/权重](https://github.com/JLChen-C/OccProphet)。

**STM4D 不入榜**：2025-09 投稿、2026-03 撤稿；纯 inflated GMO 为 **34.70 / 28.21 / 30.92**，联合 fine-grained GSO 表为 33.92/24.75/29.34，不能混用。它不是 ICLR 2026 正式论文，也不能与 IR-WM Table I 强行拼成官方榜。[OpenReview](https://openreview.net/forum?id=vfDJiI0dbu)；无 arXiv/GitHub/权重。

---

## E3. Ego-Exo4D EgoPose 与 3D Hand Forecasting

- **入口**：[Ego-Exo4D](https://ego-exo4d-data.org/) · [2026 Challenge](https://docs.ego-exo4d-data.org/challenge/) · [EgoPose 代码](https://github.com/EGO4D/ego-exo4d-egopose) · [EgoH4](https://masashi-hatano.github.io/EgoH4/) · [EggHand](https://jyoun9.github.io/EggHand)
- **必须分任务**：EgoPose 是当前帧 3D pose estimation；EgoH4/EggHand 是观测 2 秒、预测未来 1 秒的 hand forecasting。

### 2026 官方 Top 3

- **2026 challenge 实际只含 Ego-Pose Body 与 Procedure Understanding**；没有 Hand Pose，也没有 3D Hand Forecasting challenge。
- Ego-Pose Body 于 2026-05-13 关榜，但截至检索日 [CodaBench](https://www.codabench.org/competitions/15376/) 无可见 leaderboard，无法核验提交队伍和 Top 3。
- 因此不能把 2025 HP-ViT+ 或历史 PCIE/Head2Body 结果重新标成 2026 Hand/Body 冠军。

### 2026 Hand Forecasting 结果

#### EggHand（2026 间接对比；同 EgoH4 协议）

- **年份证据**：arXiv 2026-05；CVPR Findings 2026。
- **方法**：EgoVideo 视频–文本 encoder 提供 ego-motion/context，GR00T-N1.5 的 VLA action decoder 建模 hand dynamics；加几何感知损失；不依赖全身 pose 或外部 tracker。
- **数据/测评**：EgoExo4D 上复用 EgoH4 forecasting split；报告 ADE/FDE/MPJPE/MPJPE-F。
- **表现**：ADE **0.271**、FDE **0.271**、MPJPE **0.076**、MPJPE-F **0.077**。相对 EggHand 论文重训的 EgoH4（0.267/0.333/0.116/0.141），FDE -18.6%、MPJPE -34.5%，但 ADE 略差；EgoH4 原论文 ADE 0.261 属另一训练结果，不能混成同一行。
- **链接/开源**：[arXiv:2605.07642](https://arxiv.org/abs/2605.07642) · [CVPR Findings PDF](https://openaccess.thecvf.com/content/CVPR2026F/papers/Choi_EggHand_A_Multimodal_Foundation_Model_for_Egocentric_Hand_Pose_Forecasting_CVPRF_2026_paper.pdf) · [项目页](https://jyoun9.github.io/EggHand)；截至检索日无公开 GitHub/权重。

### 2026 相邻任务强方法

1. **ForeHOI（2026 间接对比）**：单目 hand-object 视频前馈完成 2D mask inpainting、3D shape completion 与 object pose，约比优化法快 100×；在 HOT3D 等数据测试，但不是未来手 pose 预测。[arXiv:2602.06226](https://arxiv.org/abs/2602.06226) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Chen_ForeHOI_Feed-forward_3D_Object_Reconstruction_from_Daily_Hand-Object_Interaction_Videos_CVPR_2026_paper.html) · [项目页](https://tao-11-chen.github.io/project_pages/ForeHOI/) · [GitHub](https://github.com/Tao-11-chen/ForeHOI)；代码/数据公开。
2. **ArtHOI（2026 间接对比）**：foundation-model priors + adaptive sampling refinement + MLLM contact reasoning，做单目 articulated hand-object 4D reconstruction；不同于 EgoPose/forecasting。[arXiv:2603.25791](https://arxiv.org/abs/2603.25791) · [项目页](https://arthoi-reconstruction.github.io/) · [GitHub](https://github.com/hitcs-zikaiwang/ArtHOI-4D-Reconstruction)；代码已发布，数据仍在补充。

---

## E4. EgoDex

- **入口**：[Apple Research](https://machinelearning.apple.com/research/egodex-learning-dexterous-manipulation) · [arXiv:2505.11709](https://arxiv.org/abs/2505.11709) · [ICLR 2026 OpenReview](https://openreview.net/forum?id=FFxkFMU89E) · [GitHub/数据与 evaluator](https://github.com/apple/ml-egodex)
- **协议**：12 hand keypoints 的 Best-of-K Average/Final 3D distance；无持续 online leaderboard。

### 2026 官方 Top 3

EgoDex 正式发表于 ICLR 2026，但官方策略表来自 2025 首发论文。按同一 **K=10 Avg Distance** 排名：

1. **EncDec + Flow Matching**：Avg/Final **0.038 / 0.041**。
2. **EncDec + DDPM**：**0.039 / 0.043**。
3. **Dec + Flow Matching**：**0.040 / 0.043**。

K=1 最优的 EncDec+BC 为 0.044/0.060，不能与 K=10 混排。这些都是 2026 正式 benchmark 的原始基线，不是 2026 新算法。

### 2026 间接 Top 3

#### 1) H-RDT（2026 正式会议；间接对比）

- **年份证据**：预印本 2025；AAAI 2026 正式论文。
- **方法/训练**：2B diffusion transformer + flow matching；先用完整 EgoDex 829h、338K+ trajectories、194 tasks、48-D hand action 预训练，再用 modular encoder/decoder 跨本体微调。
- **表现**：主报双臂机器人成功率，**没有 EgoDex Best-of-K 0.038/0.041 指标**，所以是数据利用 SOTA 而非 EgoDex 榜 SOTA。
- **链接/开源**：[arXiv:2507.23523](https://arxiv.org/abs/2507.23523) · [AAAI 2026](https://ojs.aaai.org/index.php/AAAI/article/view/38875) · [项目页](https://embodiedfoundation.github.io/hrdt) · [GitHub](https://github.com/HongzheBi/H_RDT) · [HF 权重](https://huggingface.co/embodiedfoundation/H-RDT)；代码和权重公开。

#### 2) Being-H0（2026 正式会议；间接对比）

- **年份证据**：预印本 2025；ICML 2026；代码/权重已公开。
- **方法**：physical instruction tuning + MANO/part-level motion tokenization，将 EgoDex 等人手视频预训练为 VLA，再适配机器人。
- **表现**：自建 5% EgoDex “head split” 上 Being-H0-14B/8B/1B 的 visual-grounded MPJPE 分别为 **6.87/7.20/9.71 cm**，但这不是 EgoDex 官方 Best-of-K。
- **链接/开源**：[arXiv:2507.15597](https://arxiv.org/abs/2507.15597) · [项目页](https://research.beingbeyond.com/being-h0) · [GitHub](https://github.com/BeingBeyond/Being-H0) · [HF 权重集合](https://huggingface.co/collections/BeingBeyond/being-h0-688dcc58cbd6b452f16bd7ec)；完全开源。

#### 3) Being-H0.7（2026 间接对比）

- **年份证据**：arXiv 2605.00078。
- **方法**：prior/posterior 双分支 latent world-action model；训练时用未来 observation embedding 监督 latent query，推理时不生成像素 rollout。
- **数据/表现**：EgoDex 是大规模 egocentric pretraining 来源之一；在 LIBERO/RoboCasa/CALVIN 等六个仿真 benchmark 和三类真机上验证，**没有 EgoDex trajectory distance**。
- **链接/开源**：[arXiv:2605.00078](https://arxiv.org/abs/2605.00078) · [项目页](https://research.beingbeyond.com/being-h07)；截至检索日没有独立公开训练仓/权重。

---

## R2. RoboWM-Bench

- **入口**：[项目页/完整分数](https://robowm-bench.github.io/RoboWM-Bench/) · [arXiv:2604.19092](https://arxiv.org/abs/2604.19092) · [CVPRW 2026](https://openaccess.thecvf.com/content/CVPR2026W/GigaBrainChallenge/html/Jiang_RoboWM-Bench_A_Benchmark_for_Evaluating_World_Models_in_Robotic_Manipulation_CVPRW_2026_paper.html) · [GitHub](https://github.com/fffstrong/RoboWM-Bench)
- **协议**：生成 Human/Robot manipulation video，经 retargeting 或 IDM 转成动作，在 real-to-sim 中执行；看 task/step success。

### 2026 官方 Top 3：Human track

1. **Wan 2.6（2026 官方榜）**：8 项均值 **76.6%**；各项 83/100/70/80/80/80/80/40，为 Human 第 1。闭源/产品模型；[Wan 产品页](https://wan.video/)；无可核实的 Wan2.6 完整训练仓。
2. **LVP（2026 官方榜；模型首发边界）**：均值 **47.5%**；14B latent video planner + HaMeR/MegaSAM video-to-action，LVP-1M 约 1.4M clips。预印本 2025-12，正式使用于 2026 benchmark，不计严格 2026 首发。[arXiv:2512.15840](https://arxiv.org/abs/2512.15840) · [GitHub](https://github.com/buoyancy99/large-video-planner)；开源。
3. **Veo 3.1（2026 官方榜，旧模型）**：均值约 **45.4%**；视觉质量强但 Fold Towel 为 0。[产品页](https://deepmind.google/models/veo/)；闭源。

### 2026 官方 Top 3：Robot track

1. **Cosmos-FT（2026 官方榜）**：均值 **47.5%**；8 项 90/50/50/60/40/30/40/20，为 Robot 第 1。基于 Cosmos 的 manipulation data finetuning；具体 FT 数据/权重未在 benchmark 仓完整发布。[Cosmos 开放基座](https://github.com/nvidia-cosmos/cosmos-predict2.5) · [RoboWM 论文](https://arxiv.org/abs/2604.19092)。
2. **Wan 2.6（2026 官方榜）**：均值 **22.5%**；50/20/40/40/20/10/0/0。[Wan 产品页](https://wan.video/) · [RoboWM 完整分数](https://robowm-bench.github.io/RoboWM-Bench/)；闭源。
3. **Veo 3.1（2026 官方榜，旧模型）**：均值 **10.0%**；20/20/10/20/10/0/0/0。[Veo 产品页](https://deepmind.google/models/veo/) · [RoboWM 完整分数](https://robowm-bench.github.io/RoboWM-Bench/)；闭源。

### 2026 间接强方法

1. **GigaWorld-1-Plus（2026 间接对比）**：AR-DiT + action/depth/semantic conditions；约 12,980h 异构数据；WMBench AVG 0.6834，但未跑 RoboWM execution。[arXiv:2607.02642](https://arxiv.org/abs/2607.02642) · [项目页](https://open-gigaai.github.io/giga-world-1/) · [GitHub](https://github.com/open-gigaai/giga-world-1)。
2. **Worldscape-MoE（2026 间接对比）**：camera/robot/hand 多控制 MoE，同属 action-conditioned rollout；尚无 RoboWM 分数。[arXiv:2607.03964](https://arxiv.org/abs/2607.03964) · [GitHub](https://github.com/EmbodiedCity/Worldscape-MoE.code)。
3. **Being-H0.7（2026 间接对比）**：latent future reasoning 替代像素 rollout；直接评 robot success，但没有 RoboWM 评测。[arXiv:2605.00078](https://arxiv.org/abs/2605.00078) · [项目页](https://research.beingbeyond.com/being-h07)。

---

## R3. GigaBrain Challenge 2026 World Model Track / WMBench

- **入口**：[挑战官网](https://gigaai-research.github.io/GigaBrain-Challenge-2026/) · [赛道说明](https://gigaai-research.github.io/GigaBrain-Challenge-2026/guide/world-model.html) · [HF 榜](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard) · [Baseline 仓](https://github.com/open-gigaai/CVPR-2026-Workshop-WM-Track)
- **协议**：8 个 robot tasks；action-to-video；三轮提交取 best；评 generation quality 与 WM-as-VLA-evaluator。

### 2026 官方 Top 3

1. **Team xuwu / Wan-3D（2026 官方榜）**：官网最终冠军；三轮 **50.2778 / 49.2973 / 57.1086**。完整方法论文、权重和训练代码未公开，不能从队名推断模型细节。[官网结果](https://gigaai-research.github.io/GigaBrain-Challenge-2026/) · [HF 榜](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard)。
2. **ABot-PhysWorld / AMAP CV Lab（2026 官方榜）**：亚军，第三轮 **55.9421**。基于 Wan2.1-I2V-14B DiT，以 VACE parallel context blocks 注入空间动作图；约 3M clips，使用 Qwen3-VL 生成物理检查项、Gemini 3 Pro 构造偏好，再以 LoRA Diffusion-DPO 对齐物理性。[arXiv:2603.23376](https://arxiv.org/abs/2603.23376) · [GitHub](https://github.com/amap-cvlab/ABot-PhysWorld) · [ModelScope 权重](https://www.modelscope.cn/models/amap_cvlab/Abot-PhysWorld) · [HF 榜](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard)；论文、代码、权重和训练数据已公开。
3. **Team Agent / RiceAD（2026 官方榜）**：季军；三轮 **10 / 空 / 54.3521**。提交专属架构、论文、代码和权重未公开。[官网结果](https://gigaai-research.github.io/GigaBrain-Challenge-2026/) · [HF 榜](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard)。

> 分数来自 HF 榜快照，最终名次另由挑战官网确认；挑战队缺方法报告时，只能称“官方竞赛结果”，不能称可复现论文 SOTA。

### 2026 WMBench 论文 Top 3

1. **GigaWorld-1-Plus 5B（2026 官方与间接对比）**：AVG **0.6834**；shared AR-DiT、LoRA、动作/深度/语义条件；约 12,980h 数据。[arXiv:2607.02642](https://arxiv.org/abs/2607.02642) · [项目页](https://open-gigaai.github.io/giga-world-1/) · [GitHub](https://github.com/open-gigaai/giga-world-1)；部分开源。
2. **GigaWorld-1-Nano 1.3B（2026 间接对比）**：AVG **0.6716**；面向 <24GB、>20 FPS。[arXiv:2607.02642](https://arxiv.org/abs/2607.02642) · [项目页](https://open-gigaai.github.io/giga-world-1/) · [GitHub](https://github.com/open-gigaai/giga-world-1) · [HF 权重](https://huggingface.co/open-gigaai/Giga-World-1)。
3. **Cosmos-Predict2.5 2B（2026 新榜单结果，旧模型）**：AVG **0.6123**；图像质量强、trajectory 约 0.18，说明画质不等于动作可控。[GitHub](https://github.com/nvidia-cosmos/cosmos-predict2.5)；开放。

---

## R4. EWMBench / AgiBot World Challenge @ ICRA 2026

- **入口**：[EWMBench GitHub](https://github.com/AgibotTech/EWMBench) · [论文](https://arxiv.org/abs/2505.09694) · [官方最终公告](https://www.agibot.com/article/231/detail/73.html) · [HF 榜](https://huggingface.co/spaces/agibot-world/ICRA26WM) · [官方 baseline](https://github.com/AgibotTech/AgiBotWorldChallengeICRA2026-WorldModelBaseline) · [数据](https://huggingface.co/datasets/agibot-world/AgiBotWorldChallenge-2026/tree/main/WorldModel)

### 2026 官方挑战 Top 3

1. **NeoVerse-ABot（2026 官方榜）**：中科院自动化所 + AMAP；EWMScore **0.8290**（归一化 PSNR 0.6246 / Scene 0.8974 / nDTW 0.9651），最终冠军。队伍提交与同名 NeoVerse 4D 论文不是同一已证实代码包，不能直接等同。[官方最终公告](https://www.agibot.com/article/231/detail/73.html)；无 submission-specific 论文、代码或权重。
2. **PAI@IAII / PAIWorld（2026 官方与间接对比）**：EWMScore **0.8245**（0.6161 / **0.9041** / 0.9531），最终亚军且 Scene Consistency 最高。DiT flow-matching 基座加入 Geometry-Aware Cross-View Attention、Geo-RoPE、Latent 3D-REPA，在约 2.5M 多视角 clips 上预训练。[arXiv:2606.18375](https://arxiv.org/abs/2606.18375) · [项目页](https://guhuangai.github.io/PAIWorld-Proj/) · [官方最终公告](https://www.agibot.com/article/231/detail/73.html)；截至检索日未见完整代码/权重。
3. **Loop（2026 官方榜）**：EWMScore **0.8241**（0.6207 / 0.9024 / 0.9492），最终季军。[官方最终公告](https://www.agibot.com/article/231/detail/73.html)；无 submission-specific 论文、架构、代码或权重。

### 2026 间接 Top 3

1. **EnerVerse_FT / EVAC（2026 新榜单结果，旧预印本）**：EWMBench Overall **4.7010**；AgiBot challenge val PSNR 20.9841 / SceneC 0.9013 / nDTW 0.9065。动作条件多视角生成与具身域适配；预印本 2025，不计 2026 首发。[arXiv:2505.09723](https://arxiv.org/abs/2505.09723) · [GitHub](https://github.com/AgibotTech/EnerVerse-AC) · [HF 权重](https://huggingface.co/agibot-world/EnerVerse-AC)；代码/权重公开。
2. **GigaWorld-1-Plus（2026 间接对比）**：同为 robot action-conditioned world model，但使用 WMBench 而非 EWMBench；不可直接混分。[arXiv:2607.02642](https://arxiv.org/abs/2607.02642) · [GitHub](https://github.com/open-gigaai/giga-world-1)。
3. **Worldscape-MoE（2026 间接对比）**：共享 dynamics + robot modality expert；尚无 EWMBench/挑战分。[arXiv:2607.03964](https://arxiv.org/abs/2607.03964) · [GitHub](https://github.com/EmbodiedCity/Worldscape-MoE.code)。

---

## G1. HOT3D

- **入口**：[项目页](https://facebookresearch.github.io/hot3d/) · [论文](https://arxiv.org/abs/2411.19167) · [Toolkit](https://github.com/facebookresearch/hot3d) · [BOP model-based 榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/) · [BOP model-free 榜](https://bop.felk.cvut.cz/leaderboards/modelfree-pose-detection-unseen-bop24/hot3d/) · [Hand toolkit](https://github.com/facebookresearch/hand_tracking_toolkit)
- **任务限制**：HOT3D 是 3D hand/object tracking、6DoF pose、object lifting 数据，不是 future forecasting。BOP object pose 与 hand tracking 不可混排。

### BOP model-based 当前官方 Top 3

1. **3PT-Pose（2026 正式方法；2025 官方提交）**：AP **0.553**（MSSD 0.498 / MSPD 0.608），2025-11-14 提交。CAD-prompted 3PT-D 检测 + render-and-compare 3PT-R，多视角 RGB epipolar/KDE 融合；论文为 CVPR 2026，但榜单结果发生在 2025。[CVPR 2026 PDF](https://openaccess.thecvf.com/content/CVPR2026/papers/Kalra_3D-Object_Perception_Transformer_3PT_CVPR_2026_paper.pdf) · [项目页](https://www.intrinsic.ai/publications/3pt-cvpr2026) · [官方榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/)；无公开提交专属代码/权重。
2. **3PT-Pose-H3 / IPT（旧提交）**：AP **0.513**（0.456 / 0.570），2025-10 提交；单视角 RGB-D。[官方榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/)。
3. **Co-op（MUSE，1 Hypo；旧提交）**：AP **0.401**（0.383 / 0.419），2025-10 提交；无 HOT3D 提交专属代码/权重。[官方榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/)。

HOT3D 没有名为“BOP Challenge 2026”的新比赛；上面只有 3PT 的论文年份是 2026。

### BOP model-free 当前官方 Top 3

1. **X0（2026 官方榜）**：AP **0.531**（MSSD 0.477 / MSPD 0.584），提交于 **2026-05-27**，是当前 Top 3 中唯一明确的 2026 新提交。用 onboarding 图像经 COLMAP 建物体 3DGS，DINOv3+SAM 分割，PnP+SuperRANSAC 产生 pose hypotheses，再以 GPT-2 密度置信模块评分；无论文、代码或权重。[官方榜](https://bop.felk.cvut.cz/leaderboards/modelfree-pose-detection-unseen-bop24/hot3d/)。
2. **gfreedet2-6d（旧提交）**：AP **0.489**（0.439 / 0.538），2025 提交。[官方榜](https://bop.felk.cvut.cz/leaderboards/modelfree-pose-detection-unseen-bop24/hot3d/)。
3. **gfreedet2-6d-default2d（旧提交）**：AP **0.483**（0.434 / 0.532），2025 提交；基于 3DGS onboarding 与 FoundPose 扩展，无正式论文。[官方榜](https://bop.felk.cvut.cz/leaderboards/modelfree-pose-detection-unseen-bop24/hot3d/)。

### HANDS 官方状态与 2026 间接 Top 3

- HOT3D HANDS 的公开挑战属于 ECCV/HANDS 2024；[Pose phase](https://eval.ai/web/challenges/challenge-page/2333/leaderboard/5790) 与 [Shape phase](https://eval.ai/web/challenges/challenge-page/2333/leaderboard/5792) 截至检索日均显示 “No results to show”，不存在可核验的 2026 官方 Top 3。
- 以下论文使用自建 HOT3D train/val split，**不是官方 challenge 排名，跨论文数值需谨慎**：
  1. **HandFlow（2026 间接对比）**：W-MPJPE **43.00 mm**、PA-MPJPE 5.49、WA-MPJPE 16.17；HaMeR 条件前端 + Flux-style 双流 flow-matching Transformer + MANO sequence denoising。[arXiv:2607.11221](https://arxiv.org/abs/2607.11221) · [项目/代码](https://mxxu00.github.io/HandFlow/)。
  2. **StableHand（2026 间接对比）**：W-MPJPE **57.83 mm**、PA-MPJPE **4.02**、WA-MPJPE 21.02；以四通道质量信号控制 DiT flow matching，锚定可靠观测并重生成低质量部分。[arXiv:2605.18553](https://arxiv.org/abs/2605.18553) · [项目页](https://huajian-zeng.github.io/projects/stablehand/)。
  3. **UniHand（2026 间接对比）**：W-MPJPE **63.97 mm**、PA-MPJPE 4.76、WA-MPJPE 25.24；Joint VAE 对齐 MANO/2D/3D 条件，以 latent diffusion 统一估计和生成。[arXiv:2602.21631](https://arxiv.org/abs/2602.21631) · [ICLR 2026](https://iclr.cc/virtual/2026/poster/10006863)。

---

## 2026 真正新方法索引

- **4D/世界生成**：NeoVerse、VerseCrafter、Phys4D、EvoPhys-World、WorldScape-0.2、Worldscape-MoE。
- **物理视频**：GeoPhys、WMReward、PHANTOM、CoECT、PhysRAG、NEWTON。
- **4D occupancy**：IR-WM（2025 首发、ICRA 2026 正式录用）；STM4D 为 2025 投稿、2026 撤稿，不算 2026 新方法。
- **Ego/手部**：EggHand、ForeHOI、ArtHOI、Being-H0.7；H-RDT/Being-H0 为 2025 首发但 2026 正式会议。
- **机器人世界模型**：RoboWM-Bench 的 Wan2.6/Cosmos-FT 评测、GigaWorld-1、Worldscape-MoE，以及 GigaBrain/AgiBot 竞赛队。
- **HOT3D**：X0 是 2026 新 BOP model-free 提交；HandFlow、StableHand、UniHand 是 2026 自建 HOT3D split 方法；3PT 为 CVPR 2026 正式方法但榜单提交发生在 2025。

## 仅 2026 新上榜或正式发表、模型更旧的结果

- 4DWorldBench：Diffusion as Shader、ReCamMaster、TrajectoryCrafter。
- WorldModelBench：Veo 3、Kling、Wan2.1。
- VBench-2.0：Veo 3、Vidu Q1、Wan2.1 等旧模型榜项。
- EgoDex：官方 EncDec+FM/BC/DDPM 基线。
- RoboWM：Veo 3.1、LVP 等模型首发早于 2026。
- HOT3D：3PT/Co-op 的榜单提交日期均在 2025。

## 无公开论文/代码或不可复现清单

- **EvoPhys-World**：有官方 WorldScore 榜首分数和项目演示；无论文、代码、权重和训练数据。
- **EonWorld**：有官方 WorldScore 分数；无论文、代码和权重。
- **WorldScape-0.2**：有官方 WorldScore 分数；只有 0.1 技术报告，无 0.2 论文、训练仓和权重，原官网审计时返回 404。
- **Grok Imagine Video、Veo 3.1、Wan 2.6**：闭源产品。
- **GigaBrain 的 xuwu、Agent 提交**：有官方名次/分数快照，无完整方法报告与权重；ABot-PhysWorld 已公开论文、代码、权重和数据。
- **AgiBot 的 NeoVerse-ABot、Loop 提交**：有官方名次/精确分数，无可复现提交专属仓；PAIWorld 有论文/项目页但未发布完整代码和权重。
- **HOT3D X0**：有 2026 官方 BOP 分数和方法摘要，无论文、代码或权重。
- **EggHand、PHANTOM、Phys4D**：论文/项目页已公开，代码尚未公开。
- **STM4D**：投稿已撤回，代码承诺录用后发布；不能称 ICLR 2026 论文。

## 最终审计结论

1. 15 个 `chosenls_1.md` 条目均已覆盖。
2. 所有榜单结论均区分“官方榜”“论文同协议间接测评”“相邻任务”。
3. 对找不到 2026 Top 3 的条目明确留空，没有用旧方法补齐。
4. 所有列入的方法均给出可找到的直接论文/项目/代码/权重/数据链接；不存在的链接明确写“未公开”。
5. 最值得跟进的可复现 2026 方法是：**NeoVerse、VerseCrafter、GeoPhys、WMReward、CoECT、PhysRAG、NEWTON、IR-WM、H-RDT、GigaWorld-1、ABot-PhysWorld、HandFlow、ForeHOI、ArtHOI**。

---

*生成与核验日期：2026-07-18。文件：`sota_1_2.md`。*
