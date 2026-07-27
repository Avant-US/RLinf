# chosenls_1 条目的双源 SOTA 汇总

## 检索说明

- **检索日**：2026-07-18
- **口径**：每个条目分别取 **官方榜/官方协议 Top 3** 与 **间接相关强方法 Top 3**；去重后最多 6 个方法。每个方法标注来源：`官方榜` / `间接对比` / `官方与间接对比`。
- **“SOTA”含义**：均指在**指定榜口、子集、日期**下的领先或近领先，不是跨榜绝对第一。
- **局限**：部分竞赛队伍未公开权重；部分数据集无生成 leaderboard；PhyGenBench 原论文表偏旧；实时榜以官网/HF 为准可能继续变动。
- **条目来源**：[chosenls_1.md](chosenls_1.md) → [bechmrkls_1.md](bechmrkls_1.md) 对应章节。

---

## 1. 4DWorldBench

- **入口**：[官网](https://yeppp27.github.io/4DWorldBench.github.io/) · [arXiv:2511.19836](https://arxiv.org/abs/2511.19836) · [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Lu_4DWorldBench_A_Comprehensive_Evaluation_Framework_for_3D4D_World_Generation_Models_CVPR_2026_paper.html)
- **官方口径**：论文静态表（分 Image-to-4D / Video-to-4D / Text-to-4D）；截至检索日无持续开放提交流程。

### DiffusionAsShader (DaS)（官方榜）

- **方法**：将 I2V 扩散当作“着色器”，用 3D tracking video 控制动态点上的外观；支持相机控制、运动迁移、mesh-to-video。输入：参考图 + 3D 轨迹；输出：受控视频（4D 渲染代理）。
- **解决问题**：多任务统一的 3D-aware 视频控制与 Image-to-4D 一致性。
- **数据/训练/测评**：约 <10k 视频微调（约 8×H800、3 天）；在 4DWorldBench Image-to-4D 上评测。
- **表现**：I2-4D Overall **0.763**（论文表该子榜 #1，约 2025-11）→ Image-to-4D 官方 SOTA。
- **链接**：[arXiv:2501.03847](https://arxiv.org/abs/2501.03847) · [项目页](https://igl-hkust.github.io/das/) · [GitHub](https://github.com/igl-hkust/diffusionasshader)
- **开源**：**完全开源**（代码+权重）

### ReCamMaster（官方榜）

- **方法**：在预训练 T2V 上做 video conditioning + 相机轨迹控制，按新相机重渲染单视频；UE5 多相机数据训练。
- **解决问题**：单视频相机可控的动态场景重渲染（Video-to-4D 代理）。
- **数据/训练/测评**：~136K 视频 / 13.6K 场景；4DWorldBench Video-to-4D。
- **表现**：V2-4D Overall **0.685** → Video-to-4D 官方 #1。
- **链接**：[arXiv:2503.11647](https://arxiv.org/abs/2503.11647) · [项目页](https://jianhongbai.github.io/ReCamMaster/) · [GitHub](https://github.com/KwaiVGI/ReCamMaster)
- **开源**：**部分开源**（代码/数据公开；完整效果依赖商用级 T2V 骨干说明）

### TrajectoryCrafter（官方榜）

- **方法**：双流条件扩散，对单目视频做相机轨迹重定向与高保真新视角生成。
- **解决问题**：单目视频相机轨迹编辑 / 新视角合成。
- **表现**：V2-4D Overall **0.670** → 官方 #2。
- **链接**：[项目页](https://trajectorycrafter.github.io/) , [arxiv 2503.05638](https://arxiv.org/abs/2503.05638), [Git](https://github.com/TrajectoryCrafter/TrajectoryCrafter)
- **开源**：**部分开源**

### Voyager (HunyuanWorld-Voyager)（间接对比）

- **方法**：相机条件 RGBD 视频扩散 + 实时 3D 重建，强调长程世界一致可探索场景。
- **解决问题**：可探索 3D 场景的世界一致视频生成。
- **表现**：未进 4DWorldBench 官方主表；在 WorldScore-Static 达 **77.62**（同族强方法）。
- **链接**：[arXiv:2506.04225](https://arxiv.org/abs/2506.04225) · [GitHub](https://github.com/Tencent-Hunyuan/HunyuanWorld-Voyager)
- **开源**：**完全开源**

### WonderWorld（间接对比）

- **方法**：单图交互式 3D 场景生成（显式几何 + 可导航）。
- **解决问题**：静态/可探索 3D 世界生成与相机可控漫游。
- **表现**：WorldScore-Static **72.69**；4DWorldBench 同族 3D 强基线。
- **链接**：[arXiv:2406.09394](https://arxiv.org/abs/2406.09394) · [项目页](https://kovenyu.com/wonderworld/) · [GitHub](https://github.com/KovenYu/WonderWorld)
- **开源**：**完全开源**

### Wan2.1（间接对比）

- **方法**：大规模开源视频扩散（T2V/I2V），作为 4D 世界的 2D 代理。
- **解决问题**：通用高质量视频世界仿真。
- **表现**：WorldScore / WorldModelBench / VBench 等多榜强开源基线（如 WorldModelBench Total **9.04**）。
- **链接**：[arXiv:2503.20314](https://arxiv.org/abs/2503.20314) · [GitHub](https://github.com/Wan-Video/Wan2.1)
- **开源**：**完全开源**

> 官方同表补充：CamI2V（I2-4D 0.697）、4Dfy（T2-4D 0.535）等。

---

## 2. WorldScore

- **入口**：[官网](https://haoyi-duan.github.io/WorldScore/) · [GitHub](https://github.com/haoyi-duan/WorldScore) · [arXiv:2504.00983](https://arxiv.org/abs/2504.00983) · [HF 榜](https://huggingface.co/spaces/Howieeeee/WorldScore_Leaderboard)
- **口径**：`WorldScore-Static`（可控+质量）与 `WorldScore-Dynamic`（再加运动）；3D 方法运动项常为 0。

### Voyager（官方与间接对比）

- **方法**：相机条件 RGBD 视频扩散 + 实时 3D 重建（HunyuanWorld-Voyager）。
- **表现**：Static **77.62** → WorldScore-Static 当前公开 SOTA（官网约 2025-06 起含 Voyager）。
- **链接**：[arXiv:2506.04225](https://arxiv.org/abs/2506.04225) · [GitHub](https://github.com/Tencent-Hunyuan/HunyuanWorld-Voyager)
- **开源**：**完全开源**

### WonderWorld（官方与间接对比）

- **方法**：单图交互式 3D 场景生成（FLAGS 表示 + 可导航）。
- **表现**：Static **72.69**（#2）；Camera Ctrl **92.98** 极强；Dynamic 运动项为 0。
- **链接**：[arXiv:2406.09394](https://arxiv.org/abs/2406.09394) · [项目页](https://kovenyu.com/wonderworld/) · [GitHub](https://github.com/KovenYu/WonderWorld)
- **开源**：**完全开源**（代码已 release）

### LucidDreamer（官方榜）

- **方法**：扩散先验 + 3D Gaussian/场景优化，单图建世界。
- **解决问题**：高质量静态 3D 场景重建/生成。
- **表现**：Static **70.40**（#3）；3D Consist / Photo 很高。
- **链接**：[项目页](https://luciddreamer-cvlab.github.io/) · [arXiv:2311.13384](https://arxiv.org/abs/2311.13384) · [GitHub](https://github.com/luciddreamer-cvlab/LucidDreamer)
- **开源**：**完全开源**（常见实现）

### CogVideoX-I2V（官方与间接对比）

- **方法**：Expert Transformer 视频扩散；图像+文本条件 I2V。
- **解决问题**：图像条件动态世界视频生成。
- **数据/训练/测评**：大规模视频预训练；WorldScore 动态子集。
- **表现**：Static 62.15 / Dynamic **59.12**（视频族强；非 Static 总榜 Top3）。
- **链接**：[arXiv:2408.06072](https://arxiv.org/abs/2408.06072) · [GitHub](https://github.com/THUDM/CogVideo)
- **开源**：**完全开源**

### Runway Gen-3（官方榜）

- **方法**：闭源商用 I2V/T2V。
- **表现**：Static 60.71 / Dynamic **57.58**。
- **链接**：[Runway](https://runwayml.com/)
- **开源**：**未开源**（无 arXiv/GitHub）

### LTX-Video（官方榜）

- **方法**：Lightricks 高效开源视频扩散。
- **表现**：Static 55.44 / Dynamic **56.54**。
- **链接**：[GitHub](https://github.com/Lightricks/LTX-Video) · [arXiv:2501.00103](https://arxiv.org/abs/2501.00103)
- **开源**：**完全开源**

> 注：第三方聚合站曾出现更高分条目（如 EonWorld）；本报告以官方站/论文表为准。Static 与 Dynamic 不可混排。

---

## 3. DynamicVerse

- **入口**：[官网](https://dynamic-verse.github.io/) · [GitHub](https://github.com/Dynamics-X/DynamicVerse) · [HF](https://huggingface.co/datasets/kairunwen/DynamicVerse) · [arXiv:2512.03000](https://arxiv.org/abs/2512.03000)
- **说明**：这是 **大规模动态 4D 数据集 + DynamicGen 管线**，不是生成 leaderboard。官方对比在 video depth / camera pose / intrinsics。截至检索日，“用 DynamicVerse 训生成模型并统一出榜”的文献仍少。

### DynamicGen（官方榜）

- **方法**：融合 UniDepthv2、CoTracker3、分割/检测 VFM + 动态 Bundle Adjustment，再分层生成 caption。输入：原始视频；输出：metric point maps、位姿/内参、masklets、caption。
- **解决问题**：可扩展的真实动态 4D 多模态数据生产。
- **数据/训练/测评**：互联网+公开视频；在 Sintel/KITTI/TUM-dynamics 验证几何。
- **表现**：Depth（Sintel Abs **0.205** / δ **72.9**）；Pose（Sintel ATE **0.108**，TUM ATE **0.012**）→ 论文声明几何任务整体 SOTA（约 2025-12）。
- **链接**：[arXiv:2512.03000](https://arxiv.org/abs/2512.03000) · [官网](https://dynamic-verse.github.io/) · [GitHub](https://github.com/Dynamics-X/DynamicVerse) · [HF 数据](https://huggingface.co/datasets/kairunwen/DynamicVerse)
- **开源**：**部分开源**（数据+管线；依赖多预训练组件）

### Uni4D（官方与间接对比）

- **方法**：无需重训的多阶段优化，拼接深度/跟踪/分割 VFM 做动态 4D 重建。
- **表现**：Sintel Depth Abs **0.216**；TUM ATE **0.012**（与 DynamicGen 同档）。
- **链接**：[arXiv:2503.21761](https://arxiv.org/abs/2503.21761) · [GitHub](https://github.com/Davidyao99/uni4d)
- **开源**：**完全开源**

### Metric3D / DepthCrafter（官方榜）

- **方法**：Metric3D 单帧度量深度；DepthCrafter 视频一致深度。
- **表现**：Metric3D 在 KITTI Abs **0.039** / δ **98.8** 极强；DepthCrafter Sintel Abs **0.231**。
- **链接**：
  - Metric3D v2：[arXiv:2404.15506](https://arxiv.org/abs/2404.15506) · [GitHub](https://github.com/YvanYin/Metric3D)
  - DepthCrafter：[arXiv:2409.02095](https://arxiv.org/abs/2409.02095) · [GitHub](https://github.com/Tencent/DepthCrafter)
- **开源**：**完全开源**

### MonST3R（官方与间接对比）

- **方法**：动态场景上的 DUSt3R 风格联合深度-位姿。
- **表现**：Sintel Pose ATE **0.108**；深度弱于 Uni4D/DynamicGen。
- **链接**：[arXiv:2410.03825](https://arxiv.org/abs/2410.03825) · [GitHub](https://github.com/Junyi42/monst3r)
- **开源**：**完全开源**

### UniDepth / UniDepthv2（间接对比）

- **方法**：单图度量深度+内参；DynamicGen 几何初始化组件。
- **链接**：[GitHub](https://github.com/lpiccinelli-eth/UniDepth) · [arXiv:2502.20110](https://arxiv.org/abs/2502.20110)
- **开源**：**完全开源**

### VGGT / DUSt3R（间接对比）

- **方法**：通用多视图几何基础模型；动态场景泛化有限，但是 4D 管线常用后端。
- **链接**：
  - DUSt3R：[arXiv:2312.14132](https://arxiv.org/abs/2312.14132) · [GitHub](https://github.com/naver/dust3r)
  - VGGT：[arXiv:2503.11651](https://arxiv.org/abs/2503.11651) · [GitHub](https://github.com/facebookresearch/vggt)
- **开源**：**完全开源**

---

## 4. WorldModelBench

- **入口**：[官网](https://worldmodelbench-team.github.io/) · [GitHub](https://github.com/WorldModelBench-Team/WorldModelBench) · [arXiv:2502.20694](https://arxiv.org/abs/2502.20694)
- **口径**：Judge Total Score（满分约 10；真实视频 ≈ 9.97）

### Veo 3（官方榜）

- **方法**：Google 闭源前沿 T2V/I2V。
- **解决问题**：指令遵循 + 物理 + 常识的世界建模能力评测。
- **表现**：Total **9.18** → 论文已报告最高分（约 2025）。
- **链接**：[Google DeepMind Veo](https://deepmind.google/models/veo/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Kling（官方榜）

- **方法**：快手闭源视频生成。
- **表现**：Total **9.10**。
- **链接**：[Kling AI](https://klingai.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Wan 2.1-T2V（官方与间接对比）

- **方法**：大规模开源 T2V。
- **表现**：Total **9.04** → 开源侧接近顶级闭源。
- **链接**：[arXiv:2503.20314](https://arxiv.org/abs/2503.20314) · [GitHub](https://github.com/Wan-Video/Wan2.1)
- **开源**：**完全开源**

### Minimax / Hailuo（官方榜）

- **方法**：闭源商用视频模型。
- **表现**：Total **8.92**。
- **链接**：[Hailuo / MiniMax 产品页](https://hailuoai.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### LTX-Video-T2V（官方榜）

- **方法**：Lightricks 高效开源视频扩散（T2V）。
- **表现**：Total **8.78**。
- **链接**：[arXiv:2501.00103](https://arxiv.org/abs/2501.00103) · [GitHub](https://github.com/Lightricks/LTX-Video)
- **开源**：**完全开源**

### Mochi（官方榜）

- **方法**：Genmo 开源/API 视频扩散（AsymmDiT）。
- **表现**：Mochi-official **8.66**；开源权重略低。
- **链接**：[GitHub](https://github.com/genmoai/mochi) · [HF](https://huggingface.co/genmo/mochi-1-preview)（无独立 arXiv 技术报告，以仓库声明为准）
- **开源**：**完全开源**（权重版）/ API 为部分

---

## 5. Physics-IQ

- **入口**：[官网](https://physics-iq.github.io/) · [GitHub 含双榜](https://github.com/google-deepmind/physics-IQ-benchmark) · 原论文 [arXiv:2501.09038](https://arxiv.org/abs/2501.09038)
- **推荐口径（2026-06）**：**Physics-IQ Verified**；同时可报 Original。须分 i2v vs v2v。

### Magi-1 24B + GeoPhys (BoN)（官方与间接对比）

- **方法**：MAGI-1 为自回归 chunk 视频世界模型；GeoPhys 用冻结图像编码器轨迹几何作物理 verifier，Best-of-N 选样。
- **解决问题**：物理可验的未来视频预测 / 测试时物理对齐。
- **数据/训练/测评**：MAGI 大规模视频预训练；GeoPhys 不改生成器权重，在 Physics-IQ 上 BoN。
- **表现**：Verified v2v **58.2±1.8**（2026-06-19，#1）；Original v2v **64.5%** → Verified v2v 公开 SOTA。
- **链接**：MAGI [arXiv:2505.13211](https://arxiv.org/abs/2505.13211) · [GitHub](https://github.com/SandAI-org/MAGI-1)；GeoPhys [arXiv:2606.20707](https://arxiv.org/abs/2606.20707) · [GitHub](https://github.com/ChristianInterno/GeoPhys) · [项目页](https://christianinterno.github.io/GeoPhys/)
- **开源**：二者均 **完全开源**

### Magi-1 24B（官方榜）

- **方法**：自回归 per-chunk 去噪；支持流式长视频。
- **表现**：Verified v2v **48.4±1.1**（#2）；Original 单模 v2v **56.0%**。
- **链接**：[arXiv:2505.13211](https://arxiv.org/abs/2505.13211) · [GitHub](https://github.com/SandAI-org/MAGI-1) · [HF](https://huggingface.co/sand-ai/MAGI-1)
- **开源**：**完全开源**

### Cosmos3-Super-Image2Video（官方榜）

- **方法**：NVIDIA Cosmos 世界模型套件 I2V。
- **表现**：Verified i2v **39.5±0.8**（i2v #1）；Original 上 Cosmos3-Super+WMReward 等更高。
- **链接**：[Cosmos3 技术报告 PDF](https://research.nvidia.com/labs/cosmos-lab/cosmos3/technical-report.pdf) · [cosmos-predict2.5 GitHub](https://github.com/nvidia-cosmos/cosmos-predict2.5)（开放权重变体；Super 级以报告/API 为准）
- **开源**：**部分开源**

### WMReward（官方与间接对比）

- **方法**：推理时用 V-JEPA-2 surprise 作物理奖励，对去噪轨迹 BoN/引导。
- **解决问题**：不改生成器权重的物理对齐。
- **表现**：Original：Magi-1+WMReward v2v **62.6%**；Cosmos3-Super+WMReward **63.4%**；曾报 ICCV 2025 PhysicsIQ Challenge 冠军声明。
- **链接**：[arXiv:2601.10553](https://arxiv.org/abs/2601.10553) · [GitHub](https://github.com/facebookresearch/WMReward)
- **开源**：**完全开源**

### Grok Imagine Video（官方榜）

- **方法**：xAI 闭源视频生成。
- **表现**：Verified i2v **34.8**（i2v #2，2026-06-17）。
- **链接**：[xAI / Grok 产品页](https://x.ai/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Wan 2.2 / Hunyuan Video 1.5（官方榜）

- **方法**：大规模开源/半开源视频扩散。
- **表现**：Verified i2v：Hunyuan **33.4**；Wan2.2 **32.2**；Original 上 Wan2.2+WMReward i2v **44.4%**。
- **链接**：
  - Wan2.2：[GitHub](https://github.com/Wan-Video/Wan2.2) · Wan 系列报告 [arXiv:2503.20314](https://arxiv.org/abs/2503.20314)
  - HunyuanVideo：[arXiv:2412.03603](https://arxiv.org/abs/2412.03603) · [GitHub](https://github.com/Tencent-Hunyuan/HunyuanVideo)
- **开源**：Wan **完全开源**；混元 **部分开源**（后续版本开源策略可能收紧）

> 早期原论文基线（2025）已大幅落后：VideoPoet v2v 29.5%、Gen-3 22.8%、Sora i2v 10.0% 等。

---

## 6. VBench-2.0 / VBench++

- **入口**：[VBench-2.0 项目](https://vchitect.github.io/VBench-2.0-project/) · [代码](https://github.com/Vchitect/VBench/tree/master/VBench-2.0) · [HF Leaderboard](https://huggingface.co/spaces/Vchitect/VBench_Leaderboard) · [arXiv:2503.21755](https://arxiv.org/abs/2503.21755) · VBench++ [arXiv:2411.13503](https://arxiv.org/abs/2411.13503)
- **口径**：VBench-2.0 Total（intrinsic faithfulness）；数值以下为 early-2026 公开快照，以 HF 实时榜为准。

### Veo 3（官方榜）

- **方法**：Google 闭源前沿 T2V/I2V。
- **表现**：Total **66.72%** → 公开报道 SOTA（AI Index 2026 / HF 榜快照）。
- **链接**：[Google DeepMind Veo](https://deepmind.google/models/veo/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Vidu Q1（官方榜）

- **方法**：生数科技闭源视频模型。
- **表现**：Total **62.70%**。
- **链接**：[Vidu 产品页](https://www.vidu.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Wan2.1（官方与间接对比）

- **方法**：大规模开源视频扩散（T2V/I2V）。
- **表现**：Total **61.78%** → 开源阵营前列。
- **链接**：[arXiv:2503.20314](https://arxiv.org/abs/2503.20314) · [GitHub](https://github.com/Wan-Video/Wan2.1)
- **开源**：**完全开源**

### Kling（官方榜）

- **方法**：快手闭源视频生成。
- **表现**：约 **60.2%** 档（以 HF 实时榜为准）。
- **链接**：[Kling AI](https://klingai.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Seedance 1.0 Pro（官方榜）

- **方法**：字节系视频生成。
- **表现**：约 **59.8%** 档。
- **链接**：[Seedance / 即梦相关产品页](https://jimeng.jianying.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### CogVideoX-1.5 / HunyuanVideo（官方与间接对比）

- **方法**：开源大规模 DiT 视频模型。
- **表现**：约 **55–59%** 档。
- **链接**：
  - CogVideoX：[arXiv:2408.06072](https://arxiv.org/abs/2408.06072) · [GitHub（THUDM）](https://github.com/THUDM/CogVideo) · 另见 [zai-org/CogVideo](https://github.com/zai-org/CogVideo)（后续版本开源策略可能收紧）
  - HunyuanVideo：[arXiv:2412.03603](https://arxiv.org/abs/2412.03603) · [GitHub](https://github.com/Tencent-Hunyuan/HunyuanVideo)
- **开源**：**早期版本完全开源**；1.5 及以后部分权重/代码可能不再完整开放

> VBench++ 的 Quality/Semantic 总分与 VBench-2.0 不等价，比较时需固定 T2V/I2V/Long 子集。

---

## 7. PhyGenBench

- **入口**：[官网](https://phygenbench123.github.io/) · [GitHub](https://github.com/OpenGVLab/PhyGenBench) · [arXiv:2410.05363](https://arxiv.org/abs/2410.05363) · [ICML 2025](https://proceedings.mlr.press/v267/meng25c.html)
- **口径**：PhyGenEval Average（PCA）+ Human。**注意：原论文表偏旧（约 2024–early 2025），未含 Veo/Wan/Magi；下表官方 Top3 以原表为准，间接方法建议同协议补测。**

### Runway Gen-3（官方榜）

- **方法**：闭源 T2V。
- **表现**：Avg **0.51** / Human **0.48** → 官方表 SOTA。
- **链接**：[Runway](https://runwayml.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### Kling（官方榜）

- **方法**：闭源 T2V。
- **表现**：Avg **0.49**；Optics/Thermal 分项领先。
- **链接**：[Kling AI](https://klingai.com/)（无公开 arXiv/GitHub）
- **开源**：**未开源**

### CogVideoX-5B / Vchitect 2.0（官方榜）

- **方法**：开源 T2V DiT。
- **表现**：Avg **0.45**（并列开源最强档）。
- **链接**：
  - CogVideoX：[arXiv:2408.06072](https://arxiv.org/abs/2408.06072) · [GitHub](https://github.com/THUDM/CogVideo)
  - Vchitect-2.0：[arXiv:2501.08453](https://arxiv.org/abs/2501.08453) · [GitHub](https://github.com/Vchitect/Vchitect-2.0)
- **开源**：**完全开源**

### MAGI-1（间接对比）

- **方法**：自回归 per-chunk 视频世界模型（SandAI）；物理 IQ 上显著强于早期 T2V。
- **表现**：**PhyGenBench 官方表未列**；同物理视频族强候选，建议补测。
- **链接**：[arXiv:2505.13211](https://arxiv.org/abs/2505.13211) · [GitHub](https://github.com/SandAI-org/MAGI-1) · [HF](https://huggingface.co/sand-ai/MAGI-1)
- **开源**：**完全开源**

### Wan2.1 / Wan2.2（间接对比）

- **方法**：阿里通义大规模开源视频扩散（T2V/I2V）。
- **表现**：未进 PhyGenBench 原表；WorldModelBench/VBench/Physics-IQ 物理相关维强。
- **链接**：Wan 技术报告 [arXiv:2503.20314](https://arxiv.org/abs/2503.20314) · [Wan2.1 GitHub](https://github.com/Wan-Video/Wan2.1) · [Wan2.2 GitHub](https://github.com/Wan-Video/Wan2.2)
- **开源**：**完全开源**

### GeoPhys / WMReward / PHANTOM（间接对比）

- **方法**：GeoPhys / WMReward 为推理时物理 verifier + Best-of-N；PHANTOM（CVPR 2026）在 Wan2.2-TI2V 上加物理动力学分支，用 V-JEPA2 嵌入联合预测视觉与潜物理状态。
- **解决问题**：提升生成视频物理合规（test-time 选样或训练期联合建模），而非另起全新骨干。
- **表现**：GeoPhys/WMReward 在 Physics-IQ（Verified/Original）刷新 SOTA；PHANTOM 报告 Physics-IQ / VBench-2 Physics 等物理维增益。**三者均未进入 PhyGenBench 原论文对比表**，同协议需补测。
- **链接**：
  - GeoPhys：[arXiv:2606.20707](https://arxiv.org/abs/2606.20707) · [项目页](https://christianinterno.github.io/GeoPhys/) · [GitHub](https://github.com/ChristianInterno/GeoPhys)
  - WMReward：[arXiv:2601.10553](https://arxiv.org/abs/2601.10553) · [GitHub](https://github.com/facebookresearch/WMReward)
  - PHANTOM：[arXiv:2604.08503](https://arxiv.org/abs/2604.08503) · [CVPR 2026 Open Access](https://openaccess.thecvf.com/content/CVPR2026/html/Shen_PHANTOM_Physics-Infused_Video_Generation_via_Joint_Modeling_of_Visual_and_CVPR_2026_paper.html) · [项目页](https://plan-lab.github.io/projects/phantom/)（页面标注 Code coming soon，截至检索日无公开 GitHub）
- **开源**：GeoPhys / WMReward **完全开源**；PHANTOM **论文与项目页已公开，代码尚未开源**

---

## 9. UniOcc

- **入口**：[官网](https://uniocc.github.io/) · [GitHub](https://github.com/tasl-lab/UniOcc) · [arXiv:2503.24381](https://arxiv.org/abs/2503.24381) · [ICCV 2025](https://openaccess.thecvf.com/content/ICCV2025/html/Wang_UniOcc_A_Unified_Benchmark_for_Occupancy_Forecasting_and_Prediction_in_ICCV_2025_paper.html)
- **口径**：无独立实时榜；以论文 Table 为官方结果。任务含占用预报、相机占用预测、协同占用等，**不可混排**。

### OccWorld + Voxel Flow（官方与间接对比）

- **方法**：在 OccWorld 时空 Transformer 上加 flow encoder/decoder + cross-attention，显式体素运动。
- **解决问题**：历史占用 → 未来占用预报。
- **数据/训练/测评**：UniOcc 统一 nuScenes/Waymo/CARLA/OpenCOOD；占用+体素前后向 flow。
- **表现**：nuScenes + Voxel Flow：mIoU@0s **70.64**、@1s **32.13**、@2s **22.50**、@3s **19.06** → UniOcc 预报设定官方强结果。
- **链接**：OccWorld [arXiv:2311.16038](https://arxiv.org/abs/2311.16038) · [GitHub](https://github.com/wzzheng/OccWorld)；UniOcc 增强/评测 [GitHub](https://github.com/tasl-lab/UniOcc) · [arXiv:2503.24381](https://arxiv.org/abs/2503.24381)
- **开源**：**完全开源**

### CVTOcc（官方榜）

- **方法**：沿视线采样历史帧特征建 cost volume，几何对应时序融合。
- **解决问题**：相机当前帧 3D 占用预测。
- **表现**：UniOcc Table：mIoU_geo **31.57**、IoU_geo 81.20 → **相机占用预测官方最强**。
- **链接**：[arXiv:2409.13430](https://arxiv.org/abs/2409.13430) · [GitHub](https://github.com/Tsinghua-MARS-Lab/CVT-Occ)
- **开源**：**完全开源**

### CoHFF（官方榜）

- **方法**：车联网协同语义占用，混合特征融合多车观测。
- **解决问题**：协同占用（遮挡/覆盖）。
- **表现**：mIoU_geo **34.16**、IoU_car **87.22** → 协同设定官方代表。
- **链接**：[arXiv:2402.07635](https://arxiv.org/abs/2402.07635) · [GitHub](https://github.com/rruisong/CoHFF)
- **开源**：**完全开源**

### Drive-OccWorld（间接对比）

- **方法**：视觉中心 4D 世界模型：历史 BEV + 条件归一化 memory + 动作条件世界解码；可接规划。
- **解决问题**：相机 4D 占用/flow 预报 + 端到端规划。
- **表现**：在 **Cam4DOcc 协议**上相对 OCFNet \(\tilde{\mathrm{mIoU}}_f\) **+9.4**（28.0→37.4）；尚未统一到 UniOcc 官方表。
- **链接**：[arXiv:2408.14197](https://arxiv.org/abs/2408.14197) · [项目页](https://drive-occworld.github.io/) · [GitHub](https://github.com/yuyang-cloud/Drive-OccWorld)
- **开源**：**完全开源**

### IR-WM（间接对比）

- **方法**：隐式残差世界模型：预测相对上一时刻 BEV 的残差 + 对齐校准（Drive-OccWorld 后续）。
- **解决问题**：减少静态背景冗余与长时误差累积。
- **链接**：[arXiv:2510.16729](https://arxiv.org/abs/2510.16729) · [ir-wm 分支](https://github.com/yuyang-cloud/Drive-OccWorld/tree/ir-wm)
- **开源**：**完全开源**

### OccWorld（原版，间接对比）

- **方法**：占用 token → 未来占用 + ego 轨迹（多为占用输入，非纯相机端到端）。
- **链接**：[arXiv:2311.16038](https://arxiv.org/abs/2311.16038) · [GitHub](https://github.com/wzzheng/OccWorld)
- **开源**：**完全开源**

---

## 10. Cam4DOcc

- **入口**：[CVPR 2024](https://openaccess.thecvf.com/content/CVPR2024/html/Ma_Cam4DOcc_Benchmark_for_Camera-Only_4D_Occupancy_Forecasting_in_Autonomous_Driving_CVPR_2024_paper.html) · [arXiv:2311.17663](https://arxiv.org/abs/2311.17663) · [GitHub](https://github.com/haomo-ai/Cam4DOcc)
- **口径**：论文 Table；主任务 inflated GMO，nuScenes；\(N_p=2,N_f=4\)。

### OCFNet†（全量训练）（官方榜）

- **方法**：环视序列 → 多帧体素 warp 聚合 → 占用 + 3D backward centripetal flow 多任务头。
- **解决问题**：端到端相机 4D 占用预报。
- **数据/训练/测评**：nuScenes 23930/5119；Lyft 15720/5880；8×A100，约 15 epoch。
- **表现**：nuScenes IoU_c **31.30** / IoU_f **26.82** / \(\tilde{\mathrm{IoU}}_f\) **27.98** → **官方基线榜 SOTA**。
- **链接**：[arXiv:2311.17663](https://arxiv.org/abs/2311.17663) · [GitHub](https://github.com/haomo-ai/Cam4DOcc)（含 OCFNet 配置与权重）
- **开源**：**完全开源**

### OCFNet（约 1/6 数据）（官方榜）

- **方法**：与 OCFNet† 相同架构，少数据设定。
- **表现**：nuScenes 27.86 / 23.89 / 24.77；仍高于非端到端基线。
- **链接**：[arXiv:2311.17663](https://arxiv.org/abs/2311.17663) · [GitHub](https://github.com/haomo-ai/Cam4DOcc)
- **开源**：**完全开源**

### PowerBEV-3D（官方榜）

- **方法**：PowerBEV 做 BEV 语义/实例预报，沿 z 抬升到 3D；Cam4DOcc `other_baselines` 封装。
- **表现**：nuScenes 23.08 / 21.25 / 21.86（官方第二强非 OCFNet）。
- **链接**：PowerBEV [arXiv:2306.10761](https://arxiv.org/abs/2306.10761) · [GitHub](https://github.com/EdwardLeeLPZ/PowerBEV) · 封装见 [Cam4DOcc](https://github.com/haomo-ai/Cam4DOcc)
- **开源**：**完全开源**

### Drive-OccWorld（间接对比）

- **方法**：视觉中心 4D 世界模型：历史 BEV + 条件归一化 memory + 动作条件世界解码；可接规划。
- **表现**：nuScenes inflated：\(\tilde{\mathrm{mIoU}}_f\) **37.4**、VPQ_f **25.1** → **同协议当前最强公开结果之一**（相对 OCFNet† +9.4）。
- **链接**：[arXiv:2408.14197](https://arxiv.org/abs/2408.14197) · [项目页](https://drive-occworld.github.io/) · [GitHub](https://github.com/yuyang-cloud/Drive-OccWorld)
- **开源**：**完全开源**

### IR-WM（间接对比）

- **方法**：隐式残差世界模型：预测相对上一时刻 BEV 的残差 + 对齐校准（Drive-OccWorld 后续）。
- **链接**：[arXiv:2510.16729](https://arxiv.org/abs/2510.16729) · [ir-wm 分支](https://github.com/yuyang-cloud/Drive-OccWorld/tree/ir-wm)
- **开源**：**完全开源**

### OccWorld（间接对比）

- **方法**：占用世界模型；输入多为占用而非纯相机。
- **链接**：[arXiv:2311.16038](https://arxiv.org/abs/2311.16038) · [GitHub](https://github.com/wzzheng/OccWorld)
- **开源**：**完全开源**

---

## E3. Ego-Exo4D EgoPose 与 3D Hand Forecasting 协议

- **入口**：[Ego-Exo4D](https://ego-exo4d-data.org/) · [文档](https://docs.ego-exo4d-data.org/) · [EgoH4](https://masashi-hatano.github.io/EgoH4/) · [EggHand](https://jyoun9.github.io/EggHand) · [EgoPose Challenge 代码](https://github.com/EGO4D/ego-exo4d-egopose)
- **说明**：分两支——**(A) 未来手部预报（EgoH4/EggHand）** 与 **(B) 当前帧 EgoPose Challenge**。下列合并双源；标注任务支。

### EgoH4（官方榜 · 预报）

- **方法**：Diffusion Transformer；全身姿态约束 + 可见性头 + 3D→2D 重投影；可见/不可见手均可预报。观测 2s → 预报 1s。
- **解决问题**：第一人称双手 3D trajectory + articulated pose 未来预测。
- **数据/训练/测评**：Ego-Exo4D 整理 ~156K train / 34K test；指标 ADE/FDE/MPJPE/MPJPE-F。
- **表现**：All ADE **0.261** / FDE **0.324** / MPJPE **0.115** / MPJPE-F **0.143** → EgoH4 官方表 SOTA。
- **链接**：[arXiv:2504.08654](https://arxiv.org/abs/2504.08654) · [GitHub](https://github.com/masashi-hatano/EgoH4)
- **开源**：**完全开源**（MIT，含 checkpoint）

### EgoEgoForecast（官方榜 · 预报）

- **方法**：EgoEgo 式 diffusion 预报基线（无 EgoH4 条件损失）；对照实现经 EgoH4 仓重训。
- **表现**：ADE 0.295 / FDE 0.352 / MPJPE 0.166。
- **链接**：EgoEgo 原作 [arXiv:2305.15093](https://arxiv.org/abs/2305.15093) · [GitHub](https://github.com/jyf588/EgoEgo)；EgoH4 对照 [GitHub](https://github.com/masashi-hatano/EgoH4)
- **开源**：**完全开源**（基线+对照）

### Static / CVM（官方榜 · 预报基线）

- **方法**：训练集平均姿态保持；恒速假设（协议内朴素基线）。
- **表现**：Static ADE 0.335；CVM ADE 0.346 → 说明任务远超朴素先验。
- **链接**：定义与数字见 EgoH4 [arXiv:2504.08654](https://arxiv.org/abs/2504.08654) · [GitHub](https://github.com/masashi-hatano/EgoH4)
- **开源**：协议内基线（随 EgoH4 评测代码）

### EggHand（间接对比 · 预报）

- **方法**：EgoVideo 视频–文本编码器 + GR00T-N1.5 VLA action decoder；几何感知损失。
- **解决问题**：无全身姿态/外置跟踪的可语言条件手部预报。
- **表现**：ADE 0.271、FDE **0.271**、MPJPE **0.076**、MPJPE-F **0.077**；相对 EgoH4：FDE −18.6%、MPJPE −34.5% → **关节/终点精度当前协议 SOTA**。
- **链接**：[arXiv:2605.07642](https://arxiv.org/abs/2605.07642) · [CVPR 2026F Open Access PDF](https://openaccess.thecvf.com/content/CVPR2026F/papers/Choi_EggHand_A_Multimodal_Foundation_Model_for_Egocentric_Hand_Pose_Forecasting_CVPRF_2026_paper.pdf) · [项目页](https://jyoun9.github.io/EggHand)
- **开源**：**代码截至检索日未公开 GitHub**（仅论文+项目页）

### HP-ViT+（官方榜 · EgoPose Hand Challenge）

- **方法**：ViT+CNN 加权融合手部姿态估计。
- **解决问题**：当前帧 egocentric 3D hand pose（非未来预报）。
- **表现**：Test MPJPE **24.80** / PA-MPJPE **8.31** → **2025 Hand Pose 冠军**。
- **链接**：[arXiv:2505.24411](https://arxiv.org/abs/2505.24411)（GitHub 未在论文页稳定公布）
- **开源**：**部分公开 / 挑战提交**（完整训练仓未必开放）

### HP-ViT / Multimodal SpatioTemporal Fusion（官方榜 · Pose Challenge）

- **方法**：HP-ViT 为 2024 Hand 冠军延续；Body 赛道 PCIE Multimodal SpatioTemporal Fusion MPJPE **11.25**（2025 Body 冠军）。
- **链接**：HP-ViT [arXiv:2406.12219](https://arxiv.org/abs/2406.12219) · Ego-Exo4D Challenge [文档](https://docs.ego-exo4d-data.org/challenge/) · [EgoPose 代码](https://github.com/EGO4D/ego-exo4d-egopose)
- **开源**：挑战方案，完整权重未必开放

> 官方 Hand 估计基线还包括 POTTER、HandOccNet 等；预报间接源实质增量核心为 **EggHand**。

---

## E4. EgoDex

- **入口**：[Apple Research](https://machinelearning.apple.com/research/egodex-learning-dexterous-manipulation) · [arXiv:2505.11709](https://arxiv.org/abs/2505.11709) · [GitHub](https://github.com/apple/ml-egodex)（数据+`compute_metrics.py`）
- **口径**：Best-of-K Avg/Final 3D 距离（12 关键点）；无集中在线 leaderboard，以论文 Table 2 为官方。

### EncDec + Flow Matching（官方榜）

- **方法**：X-IL EncDec 骨干 + Flow Matching 策略；历史图像/骨架/语言 → 未来 3D 手轨迹。
- **解决问题**：多模态未来灵巧手轨迹预测。
- **数据/训练/测评**：EgoDex ~725h 训练 / 7h 测试；50k steps，bs 2048，8×A100；H=2s。
- **表现**：K=10 Avg/Final **0.038 / 0.041** → **多模态采样官方 SOTA**。
- **链接**：EgoDex [arXiv:2505.11709](https://arxiv.org/abs/2505.11709) · [Apple Research](https://machinelearning.apple.com/research/egodex-learning-dexterous-manipulation) · [GitHub（数据+evaluator）](https://github.com/apple/ml-egodex)
- **开源**：**部分开源**（指标函数在 ml-egodex；完整策略训练多依赖 X-IL 引用实现）

### EncDec + BC（官方榜）

- **方法**：同 EncDec + 行为克隆（确定性）。
- **表现**：K=1 Avg/Final **0.044 / 0.060** → **确定性 K=1 官方 SOTA**。
- **链接**：[arXiv:2505.11709](https://arxiv.org/abs/2505.11709) · [GitHub](https://github.com/apple/ml-egodex)
- **开源**：**部分开源**（指标函数在 ml-egodex；完整策略训练多依赖 X-IL 引用实现）

### EncDec + DDPM（官方榜）

- **方法**：EncDec + DDPM 扩散策略。
- **表现**：K=10 0.039 / 0.043，接近 FM。
- **链接**：[arXiv:2505.11709](https://arxiv.org/abs/2505.11709) · [GitHub](https://github.com/apple/ml-egodex)
- **开源**：**部分开源**

### Dec + BC w/ goal image（官方与间接对比）

- **方法**：逆动力学：额外给定终点图像锚定中间动作。
- **表现**：Avg **0.035** / Final **0.029** → 终点误差官方最强设定之一。
- **链接**：[arXiv:2505.11709](https://arxiv.org/abs/2505.11709) · [GitHub](https://github.com/apple/ml-egodex)
- **开源**：**部分开源**

### H-RDT（间接对比）

- **方法**：在完整 EgoDex（48 维手动作）上 flow matching 预训练 → 跨本体机器人微调。
- **解决问题**：用大规模人手数据缓解机器人模仿数据稀缺。
- **表现**：主报机器人双臂成功率；**未在 EgoDex best-of-K 官方指标上对标**。
- **链接**：[arXiv:2507.23523](https://arxiv.org/abs/2507.23523) · [GitHub](https://github.com/HongzheBi/H_RDT)
- **开源**：**完全开源**

### Being-H0（间接对比）

- **方法**：大规模人手视频 VLA；MANO 级 motion tokenization；EgoDex 为 UniHand 主力源之一。
- **表现**：手部运动与机器人任务成功率；非 EgoDex Table2 协议。
- **链接**：[arXiv:2507.15597](https://arxiv.org/abs/2507.15597) · [GitHub](https://github.com/BeingBeyond/Being-H0)
- **开源**：**完全开源**（含 HF 权重）

---

## R2. RoboWM-Bench

- **入口**：[项目页](https://robowm-bench.github.io/RoboWM-Bench/) · [arXiv:2604.19092](https://arxiv.org/abs/2604.19092) · [GitHub](https://github.com/fffstrong/RoboWM-Bench)
- **口径**：生成视频 → retarget/IDM → real-to-sim 执行；**Task Success %**（Human / Robot 分轨）。

### Wan 2.6（官方榜）

- **方法**：阿里通义大规模 I2V/视频生成；零样本/少提示生成操作视频。
- **解决问题**：生成物理上更可执行的人机操作视频。
- **表现**：Human 均约 **76.6%** → **人类轨论文 SOTA**；Robot 约 22.5%。
- **链接**：[wan.video](https://wan.video) · [Wan2.1 开源仓（同系列）](https://github.com/Wan-Video/Wan2.1) · 系列报告 [arXiv:2503.20314](https://arxiv.org/abs/2503.20314) · RoboWM 评测 [arXiv:2604.19092](https://arxiv.org/abs/2604.19092)
- **开源**：**开放权重**（商业系；完整训练数据未公开；2.6 是否全量开源以官方发布为准）

### Cosmos-FT（官方榜）

- **方法**：NVIDIA Cosmos 在操作数据上微调。
- **解决问题**：缩小「通用视频 ↔ 机器人可执行」鸿沟。
- **表现**：Robot 均约 **47.5%** → **机器人轨论文 SOTA**（相对未微调 Cosmos 约 +9×）。
- **链接**：[cosmos-predict2.5 GitHub](https://github.com/nvidia-cosmos/cosmos-predict2.5) · RoboWM [arXiv:2604.19092](https://arxiv.org/abs/2604.19092)
- **开源**：**完全开源**（Apache/NVIDIA Open Model）

### Veo 3.1（官方榜）

- **方法**：Google 闭源视频世界模型。
- **表现**：Human≈45.4%；Robot≈10%（视觉强、可执行性弱于 Wan 2.6）。
- **链接**：[Google DeepMind Veo](https://deepmind.google/models/veo/) · RoboWM [arXiv:2604.19092](https://arxiv.org/abs/2604.19092)
- **开源**：**未开源**

### LVP (Large Video Planner)（官方与间接对比）

- **方法**：14B 潜扩散视频规划 + HaMeR/MegaSAM 视频→动作；专做人手交互规划。
- **数据**：LVP-1M（~1.4M clips）；RoboWM 仅 Human。
- **表现**：Human≈47.5%（Human 第 2–3）；未测 Robot。
- **链接**：[arXiv:2512.15840](https://arxiv.org/abs/2512.15840) · [GitHub](https://github.com/buoyancy99/large-video-planner)
- **开源**：**完全开源**

### Wan 2.2（官方榜）

- **方法**：Wan 系列前代。
- **表现**：Human≈39%；Robot≈5%（对照代际提升）。
- **链接**：[GitHub](https://github.com/Wan-Video/Wan2.2) · [arXiv:2503.20314](https://arxiv.org/abs/2503.20314) · RoboWM [arXiv:2604.19092](https://arxiv.org/abs/2604.19092)
- **开源**：**开放权重**

### Cosmos（未微调）（官方榜）

- **方法**：Physical AI 世界基础模型基线。
- **表现**：Human≈13.5%；Robot≈5%。
- **链接**：[cosmos-predict2.5 GitHub](https://github.com/nvidia-cosmos/cosmos-predict2.5) · RoboWM [arXiv:2604.19092](https://arxiv.org/abs/2604.19092)
- **开源**：**完全开源**

> 结论：Human 看 **Wan 2.6**；Robot 看 **Cosmos-FT**。视觉真实感 ≠ 可执行性。

---

## R3. GigaBrain Challenge 2026 World Model Track / WMBench

- **入口**：[Challenge](https://gigaai-research.github.io/GigaBrain-Challenge-2026/) · [HF 榜](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard) · [GigaWorld-1](https://open-gigaai.github.io/giga-world-1/) · [arXiv:2607.02642](https://arxiv.org/abs/2607.02642)
- **口径**：挑战 Best（三轮）；WMBench 论文 AVG（Aesthetic/Image/JEPA/Semantic/Subject/Trajectory）。

### Wan-3D v0.3 · Team xuwu（官方榜）

- **方法**：强化 3D/几何一致性的 Wan 系世界模型，服务 VLA/策略评估。
- **解决问题**：动作条件多视角 rollout 作策略评估器。
- **表现**：Best **57.11**（R3，2026-04）→ **赛道冠军 / 挑战 SOTA**。
- **链接**：[HF Leaderboard](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard) · [Challenge 官网](https://gigaai-research.github.io/GigaBrain-Challenge-2026/) · [赛道说明](https://gigaai-research.github.io/GigaBrain-Challenge-2026/guide/world-model.html)（队伍完整方法报告/权重未公开）
- **开源**：**未开源**（仅榜单名次；无独立 arXiv/GitHub）

### ABot-PhysWorld v3.0 · AMAP CV Lab（官方榜）

- **方法**：物理先验驱动的机器人世界模型。
- **表现**：Best **55.94**（R3）。
- **链接**：[HF Leaderboard](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard) · [Challenge 官网](https://gigaai-research.github.io/GigaBrain-Challenge-2026/)
- **开源**：**未开源**（无独立 arXiv/GitHub）

### RiceAD v2.0 · Team Agent（官方榜）

- **方法**：工业侧 WM 评估器方案（小米汽车）。
- **表现**：Best **54.35**（R3）。
- **链接**：[HF Leaderboard](https://huggingface.co/spaces/open-gigaai/CVPR-2026-WorldModel-Track-LeaderBoard) · [Challenge 官网](https://gigaai-research.github.io/GigaBrain-Challenge-2026/)
- **开源**：**未开源**（无独立 arXiv/GitHub）

### GigaWorld-1-Plus (5B)（官方与间接对比）

- **方法**：主办方 AR-DiT + LoRA；动作/深度/语义条件；面向长地平线评估。
- **数据**：~12,980h 异构数据；WMBench 严格 holdout。
- **表现**：AVG **0.6834** → **论文表可复现 SOTA**。
- **链接**：[arXiv:2607.02642](https://arxiv.org/abs/2607.02642) · [项目页](https://open-gigaai.github.io/giga-world-1/) · [GitHub](https://github.com/open-gigaai/giga-world-1)
- **开源**：**部分开源**

### GigaWorld-1-Nano (1.3B)（间接对比）

- **方法**：同族轻量版（<24GB / >20FPS）。
- **表现**：AVG **0.6716**（论文第 2）。
- **链接**：[arXiv:2607.02642](https://arxiv.org/abs/2607.02642) · [GitHub](https://github.com/open-gigaai/giga-world-1)
- **开源**：**开源权重**

### Cosmos-Predict2.5 (2B)（间接对比）

- **方法**：NVIDIA Video2World；强画质、弱轨迹。
- **表现**：AVG **0.6123**；Image 高但 Traj 仅约 0.18。
- **链接**：[GitHub](https://github.com/nvidia-cosmos/cosmos-predict2.5)
- **开源**：**完全开源**

---

## R4. EWMBench / AgiBot World Challenge @ ICRA 2026

- **入口**：[EWMBench GitHub](https://github.com/AgibotTech/EWMBench) · [arXiv:2505.09694](https://arxiv.org/abs/2505.09694) · [挑战](https://agibot-world.com/challenge2026) · [HF ICRA26WM](https://huggingface.co/spaces/agibot-world/ICRA26WM) · [Baseline](https://github.com/AgibotTech/AgiBotWorldChallengeICRA2026-WorldModelBaseline)
- **口径**：论文 Overall（SceneC/HSD/Dyn/nDTW/Semantics）；在线挑战 PSNR + scene_consistency + nDTW。

### NeoVerse-ABot（官方榜 · 挑战）

- **方法**：中科院自动化所 + 高德 AMAP；面向真实机器人操作动态的世界模型。
- **表现**：**ICRA 2026 WM Track 冠军**（公开报道未给精确分）。
- **链接**：[挑战官网](https://agibot-world.com/challenge2026) · [HF ICRA26WM](https://huggingface.co/spaces/agibot-world/ICRA26WM) · [官方 Baseline](https://github.com/AgibotTech/AgiBotWorldChallengeICRA2026-WorldModelBaseline) · EWMBench 协议 [arXiv:2505.09694](https://arxiv.org/abs/2505.09694)
- **开源**：**队伍权重未开源**（无独立 arXiv/GitHub）

### PAI@IAII（官方榜 · 挑战）

- **方法**：中科院工业人工智能研究院方案。
- **表现**：**亚军**。
- **链接**：[挑战官网](https://agibot-world.com/challenge2026) · [HF ICRA26WM](https://huggingface.co/spaces/agibot-world/ICRA26WM)
- **开源**：**未开源**

### Loop（官方榜 · 挑战）

- **方法**：中科大方案。
- **表现**：**季军**。
- **链接**：[挑战官网](https://agibot-world.com/challenge2026) · [HF ICRA26WM](https://huggingface.co/spaces/agibot-world/ICRA26WM)
- **开源**：**未开源**

### EnerVerse_FT / EVAC (EnerVerse-AC)（官方与间接对比）

- **方法**：具身域适配 / 动作条件多视角生成；挑战官方 baseline。
- **数据/训练/测评**：AgiBot World；EWMBench 与 challenge val。
- **表现**：论文 Overall **4.7010（SOTA）**；val: PSNR 20.98 / SceneC 0.90 / nDTW 0.91。
- **链接**：[arXiv:2505.09723](https://arxiv.org/abs/2505.09723) · [EnerVerse-AC](https://github.com/AgibotTech/EnerVerse-AC)
- **开源**：**完全开源**（含 HF）

### LTX_FT（官方榜 · 论文）

- **方法**：LTX 实时视频骨干 + 具身微调。
- **表现**：Overall **4.5493**（论文第 2）。
- **链接**：LTX 基座 [arXiv:2501.00103](https://arxiv.org/abs/2501.00103) · [GitHub](https://github.com/Lightricks/LTX-Video)；EWMBench 对比与 FT 设定见 [arXiv:2505.09694](https://arxiv.org/abs/2505.09694) · [EWMBench GitHub](https://github.com/AgibotTech/EWMBench)
- **开源**：基座 **完全开源**；具身 FT 权重视官方发布

### Ctrl-World（间接对比）

- **方法**：多视角 + 帧级动作条件 + pose memory；策略评估/改进。
- **数据**：DROID ~95k traj（非 EWMBench 官方分）。
- **表现**：可无真机排序策略；SFT 成功率 **+44.7%**。
- **链接**：[arXiv:2510.10125](https://arxiv.org/abs/2510.10125) · [GitHub](https://github.com/Robert-gyj/Ctrl-World)
- **开源**：**完全开源**

---

## G1. HOT3D

- **入口**：[官网](https://facebookresearch.github.io/hot3d/) · [arXiv:2406.09598](https://arxiv.org/abs/2406.09598) · [Toolkit](https://github.com/facebookresearch/hot3d) · [BOP HOT3D 榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/) · [HANDS MegoTrack](https://hands-workshop.org/challenge2024.html)
- **说明**：官方主任务是 **3D hand/object tracking / 6D pose**，不是 future forecasting。双源分别对应 BOP 物体姿态与手部追踪挑战/相关强方法。

### 3PT-Pose（官方榜 · BOP HOT3D）

- **方法**：检测+位姿统一 Transformer；多视角点对应 refine（f.k.a. IPT）。
- **解决问题**：egocentric 未见物体 6D detection。
- **表现**：AP **0.553**（HOT3D 现榜 SOTA，约 2025-11 起）。
- **链接**：[CVPR 2026 Open Access PDF](https://openaccess.thecvf.com/content/CVPR2026/papers/Kalra_3D-Object_Perception_Transformer_3PT_CVPR_2026_paper.pdf) · [BOP HOT3D 榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/)（公开 arXiv/GitHub 截至检索日未稳定挂出）
- **开源**：**部分公开**（方法卡/榜单结果；完整训练仓未必开放）

### 3PT-Pose-H3 / IPT（官方榜）

- **方法**：同族单视角 RGB-D 变体。
- **表现**：AP **0.513**。
- **链接**：[BOP method card / HOT3D 榜](https://bop.felk.cvut.cz/leaderboards/pose-detection-unseen-bop24/hot3d/) · 同系 CVPR 2026 3PT PDF（上条）
- **开源**：**部分公开**

### Co-op (MUSE, 1 Hypo)（官方榜）

- **方法**：少模板半稠密对应 + 概率光流 PnP。
- **表现**：AP **0.401**；约 2s/图。
- **链接**：[arXiv:2503.17731](https://arxiv.org/abs/2503.17731) · [项目页](https://naverlabs.github.io/Co-op/) · 代码入口见项目页（NAVER LABS）
- **开源**：**部分开源**（推理代码倾向公开）

### GigaPose + GenFlow（官方与间接对比）

- **方法**：BOP’24 H3 赛道获奖；GigaPose 单对应粗姿态 + GenFlow 精修。
- **表现**：HOT3D AP **0.268**（2024 挑战期 H3 最佳量级）。
- **链接**：
  - GigaPose：[arXiv:2311.14155](https://arxiv.org/abs/2311.14155) · [GitHub](https://github.com/nv-nguyen/gigapose)
  - GenFlow：[CVPR 2024 Open Access](https://openaccess.thecvf.com/content/CVPR2024/html/Moon_GenFlow_Generalizable_Recurrent_Flow_for_6D_Pose_Refinement_of_Novel_CVPR_2024_paper.html) · [arXiv:2403.11510](https://arxiv.org/abs/2403.11510)
- **开源**：GigaPose **完全开源**；GenFlow 以论文/配套实现为准

### FreeZeV2.1（间接对比）

- **方法**：BOP’24 Classic 6D 总冠军；训练无关几何+视觉基础模型 + ICP。
- **表现**：Classic AR **82.1**；HOT3D 无深度时受限，难以直接迁移。
- **链接**：[项目页](https://andreacaraffa.github.io/freeze/) · [GitHub](https://github.com/andreacaraffa/freeze) · FreeZe 论文 [arXiv:2312.00936](https://arxiv.org/abs/2312.00936)
- **开源**：**部分开源**

### HCB（官方榜 · HANDS’24 MegoTrack 手部位姿）

- **方法**：跨视特征融合 + 三角化引导 MANO + 时序平滑。
- **表现**：UmeTrack MPJPE **12.87**；Fingertip PCK AUC **70.81%** → **手部挑战第 1**。
- **链接**：[技术报告 PDF](https://hands-workshop.org/files/2024/HCB.pdf) · [HANDS’24 Challenge](https://hands-workshop.org/challenge2024.html)
- **开源**：挑战方案（完整仓未必公开）

> 相关通用方法：
> - HaMeR：[arXiv:2312.05256](https://arxiv.org/abs/2312.05256) · [GitHub](https://github.com/geopavlakos/hamer)
> - UmeTrack / toolkit：[hand_tracking_toolkit](https://github.com/facebookresearch/hand_tracking_toolkit) · HOT3D [GitHub](https://github.com/facebookresearch/hot3d)
>
> HOT3D 无深度使 RGB-D 冠军难以直接套用。

---

## 附录：仍缺公开榜 / 结果不足的条目与注意点

| 条目 | 情况 |
|------|------|
| **DynamicVerse** | 无生成 leaderboard；官方仅几何/标注质量对比 |
| **EgoDex** | 有论文 baselines + evaluator，无持续在线总榜 |
| **Ego-Exo4D Hand Forecasting** | EgoH4/EggHand 为后续协议，非 EgoVis 官方总榜 |
| **PhyGenBench** | 官方表偏旧；新模型需同协议补测 |
| **GigaBrain / AgiBot 挑战队** | 冠亚军多不公开完整权重；复现优先官方 baseline |
| **4DWorldBench** | 静态论文表；少见开放自助提交 |
| **HOT3D** | 跟踪/姿态榜，非 forecasting；间接“未来预测”方法不适用本条官方口径 |

### 跨条目高频“常客”模型（便于横向对照）

| 模型族 | 常出现条目 | 角色 |
|--------|------------|------|
| Wan 系 | WorldScore/WorldModelBench/VBench/Physics-IQ/RoboWM/GigaBrain | 开源视频/世界模型骨干 |
| Veo / Kling / Gen-3 | 多视频物理/世界榜 | 闭源前沿 |
| MAGI-1 + GeoPhys/WMReward | Physics-IQ（及物理增强） | 物理未来预测 + test-time verifier |
| Drive-OccWorld / IR-WM | UniOcc 间接、Cam4DOcc | 相机 4D occupancy forecasting |
| EnerVerse / GigaWorld-1 | EWMBench、WMBench | 具身动作条件世界模型 |
| Voyager / WonderWorld | WorldScore、4D 间接 | 可探索 3D 世界生成 |

---

*生成日期：2026-07-18 · 输出文件：`sota_1.md` · 中间草稿（可删）：`sota_models_7bench_2026-07.md`*
