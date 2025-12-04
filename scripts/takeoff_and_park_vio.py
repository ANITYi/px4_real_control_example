#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# 全局变量
current_state = State()
current_pose = PoseStamped() 
last_vision_time = rospy.Time(0)
vision_data_received = False

# 状态回调
def state_cb(msg):
    global current_state
    current_state = msg

# 本地位置回调
def pos_cb(msg):
    global current_pose
    current_pose = msg

# 视觉定位回调
def vision_cb(msg):
    global last_vision_time, vision_data_received
    last_vision_time = rospy.Time.now()
    vision_data_received = True

def main():
    rospy.init_node('takeoff_with_rc_override', anonymous=True)

    # 1. 订阅
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pos_cb)
    vision_sub = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, vision_cb)
    
    # 2. 发布位置指令
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # 3. 新增：发布 RC 模拟信号
    rc_override_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
    
    # 4. 服务
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20) # 20Hz

    # 等待 FCU 连接
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    # 设定起飞目标
    target_alt = 1.5
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = target_alt

    # 准备 RC Override 消息 (模拟摇杆回中)
    # 通道顺序通常是: [Roll, Pitch, Throttle, Yaw, Mode, ...]
    # 1500 是 PWM 的中间值，代表摇杆在正中间
    # 在 POSCTL 模式下，油门(Throttle) 1500 意味着“保持当前高度”
    rc_msg = OverrideRCIn()
    rc_msg.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0] 
    # 注意：如果你的飞控通道映射不同（例如油门是通道1），请相应调整数组顺序

    # 发送一些设定点
    for i in range(100):   
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rc_override_pub.publish(rc_msg) # 开始发送 RC 信号以建立连接
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    rospy.loginfo("Starting takeoff sequence...")
    
    posctl_switched = False

    while not rospy.is_shutdown():
        # ------------------------------------------
        # 1. 持续发送 RC Override 信号 (核心)
        # ------------------------------------------
        # 无论什么模式，持续告诉飞控“摇杆在中间”，防止 RC Loss
        rc_override_pub.publish(rc_msg)

        # ------------------------------------------
        # 2. 起飞阶段 (OFFBOARD)
        # ------------------------------------------
        if not posctl_switched:
            # 维持 OFFBOARD 和 解锁
            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("Offboard enabled")
                last_req = rospy.Time.now()
            else:
                if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            # 发布位置设定点
            local_pos_pub.publish(pose)

            # 检查是否到达高度
            if current_state.mode == "OFFBOARD" and current_state.armed:
                current_z = current_pose.pose.position.z
                if abs(current_z - target_alt) < 0.1:
                    # 检查视觉
                    if vision_data_received and (rospy.Time.now() - last_vision_time) < rospy.Duration(0.5):
                        rospy.loginfo("Target reached & Vision good. Switching to POSCTL (Simulated RC).")
                        
                        # 切换到 POSCTL
                        posctl_mode = SetModeRequest()
                        posctl_mode.custom_mode = 'POSCTL'
                        if set_mode_client.call(posctl_mode).mode_sent:
                            rospy.loginfo("Switched to POSCTL success!")
                            posctl_switched = True
        
        # ------------------------------------------
        # 3. 悬停阶段 (POSCTL + Simulated RC)
        # ------------------------------------------
        else:
            # 此时我们在 POSCTL 模式
            # 只要 rc_msg (1500, 1500, 1500, 1500) 持续发送
            # 无人机就会利用视觉定位保持在当前位置
            rospy.loginfo_throttle(2, "Hovering in POSCTL with Virtual RC...")
            
            # 如果外部脚本修改了模式 (比如切到 LAND)，这个循环依然跑着发 RC 信号，
            # 这通常是安全的，因为 RC 中位信号不会干扰自动降落。

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass