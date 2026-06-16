* 问运维(田晨红或运维服务台)能不能像以前那样给H200挂载一个十几T的SSD或者RAID到该H200, 可以则问相关命令, 然后挂载到 /mnt/r/
* 把谷歌存储桶 /mnt/g/VENV/H2/VENV/rlinf_venv/ 复制到 /mnt/r/VENV/rlinf_venv/ 这是 RLinf运行的python虚拟环境
* 到 /home/physical/SRC/RL/RLinf 把git的branch改到 b0529wan
* 激活 /mnt/g/VENV/H2/VENV/rlinf_venv/ 然后执行 ray stop
* RLinf中执行 b\trn\r1\prepare\start_ray.sh
* 把RLinf 中的 b\x\R1PR\env.sh 里的各种路径相关的环境变量改成对应的路径
* 把 b\x\R1PR\R2 复制成 b\x\R1PR\R3
* 修改R3下的yaml和sh文件为你想训练的样子, 如果想把视频loss权重调低, 可能要加一个--loss.lambda_video 然后值为小于1的值
* 执行 R3 中的 sh 文件. console 会打印日志的地址, 从日志中可以找到 wandb 的地址. 如果用你的wandb, 记得把project设为public, 这样我才能看到.