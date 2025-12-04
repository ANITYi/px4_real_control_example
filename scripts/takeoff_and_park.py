#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# 全局变量
current_state = State()
current_pose = PoseStamped() 
last_vision_time = rospy.Time(0) # 新增：用于记录上一次收到视觉定位的时间
vision_data_received = False     # 新增：标志位

# 状态回调
def state_cb(msg):
    global current_state
    current_state = msg

# 本地位置回调
def pos_cb(msg):
    global current_pose
    current_pose = msg

# 新增：视觉定位回调
def vision_cb(msg):
    global last_vision_time, vision_data_received
    # 更新最后一次收到视觉数据的时间
    last_vision_time = rospy.Time.now()
    vision_data_received = True

def main():
    rospy.init_node('takeoff_script', anonymous=True)

    # 1. 订阅状态
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    
    # 2. 订阅本地位置信息
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pos_cb)

    # 3. 新增：订阅视觉定位话题 (用于判断是否有视觉数据)
    vision_sub = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, vision_cb)
    
    # 4. 发布位置指令
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # 服务客户端
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20) # 20Hz

    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    # 设定目标高度
    target_alt = 1.5
    
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = target_alt

    # 发送一些设定点以允许切换到 OFFBOARD
    for i in range(100):   
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    rospy.loginfo("Starting takeoff sequence...")

    while not rospy.is_shutdown():
        # 1. 尝试切换 OFFBOARD 和 解锁
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("Offboard enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        
        # 持续发布位置指令 (保持在 OFFBOARD 模式下的控制)
        local_pos_pub.publish(pose)

        # 2. 高度判断与模式切换逻辑
        if current_state.mode == "OFFBOARD" and current_state.armed:
            # 获取当前高度
            current_z = current_pose.pose.position.z
            err_z = abs(current_z - target_alt)
            
            # 判定到达：误差小于 0.08 米
            if err_z < 0.08:
                # 检查视觉数据是否正常
                # 逻辑：如果有收到过数据，且最后一条数据是在 0.5 秒内收到的 (保证数据实时性)
                is_vision_alive = vision_data_received and (rospy.Time.now() - last_vision_time) < rospy.Duration(0.5)

                if is_vision_alive:
                    rospy.loginfo("Target altitude reached. Vision data detected. Switching to POSCTL.")
                    
                    posctl_mode = SetModeRequest()
                    posctl_mode.custom_mode = 'POSCTL'
                    
                    if set_mode_client.call(posctl_mode).mode_sent:
                        rospy.loginfo("Switched to POSCTL. Hovering based on Vision.")
                        break # 任务完成，退出循环
                else:
                    # 如果没有视觉数据，或者数据断流，为了安全，不切换模式，保持 OFFBOARD 悬停并打印警告
                    rospy.logwarn_throttle(1, "Target reached, but NO valid vision data stream! Staying in OFFBOARD.")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass