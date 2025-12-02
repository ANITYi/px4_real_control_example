#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('takeoff_script', anonymous=True)

    # 订阅状态，用于判断是否连接和当前模式
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    
    # 发布位置指令
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # 服务：解锁和切换模式
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20) # 20Hz

    # 等待连接到飞控
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    # 创建目标位置：悬停在 Z=2 米
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # --- 关键点 ---
    # 在切换到 OFFBOARD 模式之前，必须先发送一些设定点
    for i in range(100):   
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    # 切换到 OFFBOARD 模式
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # 解锁指令
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    # 起飞逻辑
    took_off = False

    while not rospy.is_shutdown():
        # 1. 尝试切换 OFFBOARD 和 解锁
        if not took_off:
            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("Offboard enabled")
                last_req = rospy.Time.now()
            else:
                if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()
        
        # 持续发布位置指令让飞机飞上去
        local_pos_pub.publish(pose)

        # 简单判断高度（实际应用中建议订阅 local_position/pose 来判断）
        # 这里我们假设运行 10秒后肯定到了2米
        if current_state.mode == "OFFBOARD" and current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(10.0):
            rospy.loginfo("Target altitude reached. Switching to AUTO.LOITER (Hold).")
            
            # 切换到 AUTO.LOITER (悬停模式)
            # 注意：PX4 中 AUTO.LOITER 不需要外部指令，飞控自己会定在那
            loiter_mode = SetModeRequest()
            loiter_mode.custom_mode = 'AUTO.LOITER'
            
            if set_mode_client.call(loiter_mode).mode_sent:
                rospy.loginfo("Switched to AUTO.LOITER. Exiting script.")
                took_off = True
                break # 退出循环

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass