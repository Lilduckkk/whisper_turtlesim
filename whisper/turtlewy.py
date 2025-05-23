import subprocess
import whisper
import os
import rospy
from geometry_msgs.msg import Twist


class TurtleVelocityPublisher:
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
    

    def publish_velocity_once(self):
        print("发布速度时候的指令:",self.zhiling)
        """发布一次速度命令到海龟"""
        # 创建Twist消息实例
        if "前进"  in self.zhiling:   
            self.move_forward()
        elif "后退"  in self.zhiling:
            self.move_backward()
        elif "左转" in self.zhiling:
            self.turn_left()
        elif "右转" in self.zhiling:
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
        self.publish_velocity(twist_msg)
        rospy.loginfo("左转")

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -1.57  # 右转约90度
        self.publish_velocity(twist_msg)
        rospy.loginfo("右转")

    def run(self):
        """运行方法，调用录音和转录"""
        self.record_audio(duration=3, rate=48000)
        self.transcribe_audio()

if __name__ == '__main__':
    try:
        # 创建类的实例并发布速度
        velocity_publisher = TurtleVelocityPublisher()
        # velocity_publisher.run()
        velocity_publisher.publish_velocity_once()
    except rospy.ROSInterruptException:
        # 处理ROS中断异常
        pass


