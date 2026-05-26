# bt260415分支
从commit 64bf29a30a5920b83b3e9633c35013d9abeea2e0 分支出来
分支对比: https://github.com/Avant-US/RLinf/compare/d7ed6e0973e53612d3644ea3f3a2d08eb1999972..64bf29a30a5920b83b3e9633c35013d9abeea2e0

# bt/docs/ov
对RLinf代码和论文的分析

# bt/docs/simRL
RLinf对接仿真环境的设计, 由于仿真团队暂时没时间协助, 暂时只有设计

# bt/docs/rwRL
RLinf与真机对接做online RL的设计. 改动和新增了哪些代码, 也在设计文档中有描述.
整体设计: bt\docs\rwRL\r1pro5op47.md 和 bt\docs\rwRL\r1pro6op47.md, 并实现了整体设计的一部分: bt\docs\rwRL\r1pro5op47_imp1.md, 主要实现了R1 Pro的接入所需要的各个类
安全设计: bt\docs\rwRL\safety_2_joinlimit_3.md 主要是为了限制各关节和EE在一定范围内活动, 对关节的限制已测试通过, 但对EE的限制虽实现但尚未测试
用online RL实现Single Arm Reach任务的设计文档: bt\docs\rwRL\r1pro6op47_reach_joint3_3.md 已测试.
可以在命令行控制各关节的工具的设计文档: bt\docs\rwRL\test_galaxea_r1_pro_cli_controller.md 已测试, 改工具一般用于看看指定的关节角度或位姿放到真机身上会不会太离谱或有危险.

# bt0331分支
从commit db94487c2245f57844a98df9abe8ab7f11732f9e 分支出来
与StarVLA的GR00T集成, 代码与文档均在: bt\lerobot_str_groot 文件夹中. 已测试.
