import subprocess
import whisper
import os
import rospy
from geometry_msgs.msg import Twist


class Whisper_Turtle:
    def __init__(self):
        """初始化TurtleVelocityPublisher类，设置ROS节点和发布者"""
        # 初始化ROS节点，添加anonymous=True使节点名称唯一
        rospy.init_node('turtle_velocity_publisher', anonymous=True)
        
        # 创建一个发布者，发布Twist消息到/turtle1/cmd_vel话题
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # 设置指令
        self.zhiling = ""

        # 等待一小段时间让发布者连接到话题
        rospy.sleep(0.5)
    
    def record_audio(self,duration=3, rate=16000, output_file="output.wav"):
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


    def transcribe_audio(self):
        # ==== 1. 加载模型 ====
        model = whisper.load_model("small")  # 可选 tiny, base, small, medium, large

        # ==== 2. 输入音频文件路径 ====
        audio_path = "output.wav"  # 替换为你的音频文件路径

        # ==== 3. 执行转录（自动语言识别 + 分段） ====
        print("开始转录...")
        result = model.transcribe(audio_path, verbose=False, language="zh")  # 指定语言为中文

        # ==== 4. 打印整体文本 ====
        print("\n=== 转录完成的全文 ===")
        print(result["text"])

        # ==== 5. 分段打印（含时间戳） ====
        print("\n=== 含时间戳的分段文本 ===")
        for segment in result["segments"]:
            print(f"[{segment['start']:.2f} - {segment['end']:.2f}]: {segment['text']}")

        # ==== 6. 保存为 TXT 文件 ====
        output_file = "transcription_result.txt"
        with open(output_file, "w", encoding="utf-8") as f:
            f.write("完整转录文本：\n")
            f.write(result["text"] + "\n\n")
            f.write("分段内容（含时间戳）：\n")
            for segment in result["segments"]:
                f.write(f"[{segment['start']:.2f} - {segment['end']:.2f}]: {segment['text']}\n")

        self.zhiling = result["text"]
        print("指令:",self.zhiling)
        print(f"\n✅ 转录结果已保存至：{output_file}")

    def publish_velocity_once(self):
        print("发布速度时候的指令:",self.zhiling)
        """发布一次速度命令到海龟"""
        # 创建Twist消息实例
        if "前进" in self.zhiling or "前進" in self.zhiling:    
            self.move_forward()
        elif "后退" in self.zhiling or "後退" in self.zhiling:
            self.move_backward()
        elif "左转" in self.zhiling or "左轉" in self.zhiling:
            self.turn_left()
        elif "右转" in self.zhiling or "右轉" in self.zhiling:
            self.turn_right()
        else:
            print("指令不明确")
            return
        # 打印日志信息
        # rospy.loginfo("Published velocity: Linear.x = 2.0, Angular.z = 0.0")

    def move_forward(self):
        """前进"""
        twist_msg = Twist()
        twist_msg.linear.x = 2.0
        self.pub.publish(twist_msg)
        rospy.loginfo("前进")

    def move_backward(self):
        twist_msg = Twist()
        twist_msg.linear.x = -2.0  # 后退速度
        self.pub.publish(twist_msg)
        rospy.loginfo("后退")
    
    def turn_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = 1.57  # 左转约90度
        self.pub.publish(twist_msg)
        rospy.loginfo("左转")

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -1.57  # 右转约90度
        self.pub.publish(twist_msg)
        rospy.loginfo("右转")

    def run(self):
        """运行方法，调用录音和转录"""
        self.record_audio()
        self.transcribe_audio()
        self.publish_velocity_once()
        
if __name__ == '__main__':
    try:
        # 创建类的实例并发布速度
        node = Whisper_Turtle()
        node.run()
    except rospy.ROSInterruptException:
        # 处理ROS中断异常
        pass


