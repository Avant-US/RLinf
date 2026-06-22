import torch
import os
import sys

def main():
    # 强制将 stdout 重新配置为行缓冲 (遇到 \n 即刷新)，保证日志零延迟
    sys.stdout.reconfigure(line_buffering=True)
    
    world_size = int(os.environ.get("WORLD_SIZE", 1))
    rank = int(os.environ.get("RANK", 0))
    gpu_count = torch.cuda.device_count()
    
    print(f"--- Node Rank: {rank}/{world_size} ---")
    print(f"Detected GPUs on this node: {gpu_count}")
    
    for i in range(gpu_count):
        print(f"GPU {i}: {torch.cuda.get_device_name(i)}")

if __name__ == "__main__":
    main()

