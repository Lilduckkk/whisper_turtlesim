import subprocess

def record_audio(duration=10, rate=48000, output_file="output.wav"):
    """调用 arecord 命令录制音频"""
    command = [
        "arecord",
        "-D", "pulse",       # 使用 PulseAudio 设备
        "-d", str(duration), # 录制时长（秒）
        "-r", str(rate),     # 采样率
        "-t", "wav",         # 文件格式
        output_file          # 输出文件名
    ]
    
    try:
        print(f"开始录音 {duration} 秒...")
        # 执行命令并等待完成
        subprocess.run(command, check=True)
        print(f"录音完成，文件已保存为: {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"录音失败: {e}")
    except FileNotFoundError:
        print("错误: arecord 命令未找到，请确保已安装 alsa-utils,安装指令:sudo apt-get install alsa-utils")

# 使用示例
if __name__ == "__main__":
    record_audio(duration=3, rate=48000)
