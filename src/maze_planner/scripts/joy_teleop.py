#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝牙手柄遥操作节点
将 /joy (sensor_msgs/Joy) 消息转换为 /cmd_vel_in (geometry_msgs/Twist) 速度指令，
供 teach_repeat_node 记录第一圈手柄控制轨迹。

支持的手柄映射（默认 Xbox/PS 布局）：
  - 左摇杆 Y轴 (axis 1): 前进/后退 (前推正转，后拉倒车)
  - 左摇杆 X轴 (axis 0): 左右转向
  - 或者右摇杆 X轴 (axis 2): 左右旋转
  - L2 (axis 2 或 axis 4): 减速
  - R2 (axis 5): 加速倍率
  - 十字键 (Hat): 微调方向

参数:
  ~axis_linear     (int, default: 1)    前后轴索引
  ~axis_angular    (int, default: 0)    左右轴索引
  ~scale_linear    (double, default: 0.5)  最大线速度 (m/s)
  ~scale_angular   (double, default: 1.0)  最大角速度 (rad/s)
  ~deadzone        (double, default: 0.05) 死区阈值
  ~invert_linear   (bool, default: false)  是否反转线速度轴
  ~invert_angular  (bool, default: false)  是否反转角速度轴
  ~cmd_topic       (str, default: "/cmd_vel_in")  发布的速度话题
  ~joy_topic       (str, default: "/joy")         订阅的手柄话题
  ~button_deadman  (int, default: -1)  死手柄关按钮 (按住才发送指令, -1 禁用)
  ~button_mode_switch (int, default: -1)  模式切换按钮 (-1 禁用)
  ~hat_deadman     (int, default: -1)  十字键作为死手柄关 (-1 禁用)
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTeleop:
    def __init__(self):
        self.nh = rospy

        # --- 读取参数 ---
        self.axis_linear = rospy.get_param("~axis_linear", 1)       # 左摇杆Y
        self.axis_angular = rospy.get_param("~axis_angular", 0)     # 左摇杆X
        self.scale_linear = rospy.get_param("~scale_linear", 0.5)   # 最大前进速度 m/s
        self.scale_angular = rospy.get_param("~scale_angular", 1.0) # 最大角速度 rad/s
        self.deadzone = rospy.get_param("~deadzone", 0.05)
        self.invert_linear = rospy.get_param("~invert_linear", False)
        self.invert_angular = rospy.get_param("~invert_angular", False)
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_in")
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.button_deadman = rospy.get_param("~button_deadman", -1)
        self.button_mode_switch = rospy.get_param("~button_mode_switch", -1)
        self.hat_deadman = rospy.get_param("~hat_deadman", -1)

        # 加速倍率: 可以用 R2 (axis 5) 控制
        self.axis_speed_scale = rospy.get_param("~axis_speed_scale", -1)  # 加速轴索引, -1 禁用
        self.speed_scale_min = rospy.get_param("~speed_scale_min", 0.3)
        self.speed_scale_max = rospy.get_param("~speed_scale_max", 1.0)

        # --- 内部状态 ---
        self.last_joy = None
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.joy_cb, queue_size=5)

        # 使用定时器以固定频率发布 (避免手柄断连时小车一直跑)
        self.rate = rospy.Rate(20)  # 20Hz
        self.last_joy_time = rospy.Time.now()
        self.timeout = 0.3  # 300ms 超时自动停车

        # 死手柄关按钮状态
        self.deadman_pressed = False
        # 如果是 hat 死手柄关，记录 hat 状态
        self.hat_active = False

        rospy.loginfo("JoyTeleop 节点已启动")
        rospy.loginfo("  话题: %s -> %s", self.joy_topic, self.cmd_topic)
        rospy.loginfo("  最大线速度: %.2f m/s, 最大角速度: %.2f rad/s",
                      self.scale_linear, self.scale_angular)

    def joy_cb(self, msg):
        self.last_joy = msg
        self.last_joy_time = rospy.Time.now()

        # 检查死手柄关按钮
        if self.button_deadman >= 0 and self.button_deadman < len(msg.buttons):
            self.deadman_pressed = (msg.buttons[self.button_deadman] == 1)

        # 检查 hat 死手柄关
        if self.hat_deadman >= 0:
            # hat_deadman 作为 hat 索引，检查是否推上
            if len(msg.axes) > self.hat_deadman:
                self.hat_active = (abs(msg.axes[self.hat_deadman]) > 0.5)

    def compute_cmd(self):
        """从最近的手柄状态计算速度指令"""
        cmd = Twist()

        if self.last_joy is None:
            return cmd

        axes = self.last_joy.axes
        buttons = self.last_joy.buttons

        # 死手柄关逻辑：如果有死手柄关按钮，必须按住才输出
        if self.button_deadman >= 0:
            if not self.deadman_pressed:
                return cmd

        if self.hat_deadman >= 0:
            if not self.hat_active:
                return cmd

        # 线速度 (前后)
        linear_val = 0.0
        if abs(self.axis_linear) < len(axes):
            linear_val = axes[self.axis_linear]
            if self.invert_linear:
                linear_val = -linear_val
            if abs(linear_val) < self.deadzone:
                linear_val = 0.0

        # 角速度 (左右)
        angular_val = 0.0
        if abs(self.axis_angular) < len(axes):
            angular_val = axes[self.axis_angular]
            if self.invert_angular:
                angular_val = -angular_val
            if abs(angular_val) < self.deadzone:
                angular_val = 0.0

        # 速度倍率 (用加速轴或按钮)
        speed_scale = 1.0
        if self.axis_speed_scale >= 0 and abs(self.axis_speed_scale) < len(axes):
            raw = axes[self.axis_speed_scale]
            # 通常扳机键: -1 是松开, 1 是按下
            # 映射到 [speed_scale_min, speed_scale_max]
            t = (raw + 1.0) / 2.0  # 映射到 0~1
            t = max(0.0, min(1.0, t))
            speed_scale = self.speed_scale_min + t * (self.speed_scale_max - self.speed_scale_min)

        cmd.linear.x = linear_val * self.scale_linear * speed_scale
        cmd.angular.z = angular_val * self.scale_angular * speed_scale

        return cmd

    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            # 检查是否超时 (手柄断连)
            dt = (rospy.Time.now() - self.last_joy_time).to_sec()
            if dt > self.timeout:
                # 超时，发停止指令
                cmd = Twist()
            else:
                cmd = self.compute_cmd()

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("joy_teleop", anonymous=False)
    node = JoyTeleop()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
