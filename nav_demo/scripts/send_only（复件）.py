#!/usr/bin/env python
# encoding: utf-8

# """
# ROS节点脚本，用于从CSV文件中读取目标位置和方向数据，
# 并以设定的频率发布到'/send_mark_goal'主题。

# CSV文件格式应为：
# x, y, z, qx, qy, qz, qw
# 其中，(x, y, z)表示目标位置坐标，(qx, qy, qz, qw)表示目标方向的四元数。

# 注意：确保CSV文件编码为UTF-8，以避免中文注释或数据读取异常。

# """

# import rospy
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import quaternion_from_euler
# import math
# import csv

# def read_goal_data_from_file(filename):
#     """
#     从指定的CSV文件中逐行读取目标位置和方向数据。
#     返回一个生成器，每次迭代返回一行目标位置和方向数据的浮点数列表。

#     参数:
#     filename (str): 包含目标数据的CSV文件路径

#     返回:
#     generator: 一个生成器，每次迭代返回一行目标位置和方向数据的浮点数列表
#     """
#     with open(filename, 'r', newline='', encoding='utf-8') as csvfile:
#         reader = csv.reader(csvfile, delimiter=',')
#         next(reader)  # 跳过表头（如果有）

#         while True:
#             try:
#                 row = next(reader)
#             except StopIteration:
#                 # 文件已读完或为空，结束循环
#                 break

#             # 将读取到的数据转换为浮点数并返回
#             yield [float(value) for value in row]


# def send_goal_to_move_base(filename):
#     """
#     从指定的CSV文件中读取目标位置和方向数据，并以设定的频率发布到'/send_mark_goal'主题。

#     参数:
#     filename (str): 包含目标数据的CSV文件路径
#     """
#     # 初始化ROS节点
#     rospy.init_node('send_goal_node', anonymous=True)

#     # 创建一个发布者，用于发布目标点
#     goal_pub = rospy.Publisher('/send_mark_goal', PoseStamped, queue_size=1)

#     rate = rospy.Rate(1)  # 设置发布频率为每秒1次

#     for goal_data in read_goal_data_from_file(filename):
#         if not goal_data:
#             # 文件已读完或为空，跳过本次循环
#             continue
        
#         # 更新目标点的位姿信息
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'map'
#         goal_pose.header.stamp = rospy.Time.now()

#         goal_pose.pose.position.x = goal_data[0]
#         goal_pose.pose.position.y = goal_data[1]
#         goal_pose.pose.position.z = goal_data[2]
#         goal_pose.pose.orientation.x = goal_data[3]
#         goal_pose.pose.orientation.y = goal_data[4]
#         goal_pose.pose.orientation.z = goal_data[5]
#         goal_pose.pose.orientation.w = goal_data[6]

#         print(goal_data[0])
#         rate.sleep()
#         rospy.sleep(0.1)
#         # 发布目标点
#         goal_pub.publish(goal_pose)

#         # 等待指定的发布间隔时间
        
#         rospy.sleep(0.1)


# if __name__ == '__main__':
#     try:
#         target_data_file = "/home/robot/Ros_Test/Test_07/src/nav_demo/scripts/data.csv"  # 替换为实际文件路径
#         send_goal_to_move_base(target_data_file)
#     except rospy.ROSInterruptException:
#         pass

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math
import csv

def angles_to_quaternion(yaw, pitch, roll):
    """
    将给定的欧拉角（yaw、pitch、roll）转换为对应的四元数（qx、qy、qz、qw）。

    参数:
    yaw (float): 偏航角（绕Z轴旋转的角度，单位：弧度）
    pitch (float): 俯仰角（绕Y轴旋转的角度，单位：弧度）
    roll (float): 翻滚角（绕X轴旋转的角度，单位：弧度）

    返回:
    tuple[float]: 四元数 (qx, qy, qz, qw)，表示旋转姿态
    """
    return quaternion_from_euler(roll, pitch, yaw, axes='rzyx')

def read_and_convert_csv(filename):
    """
    从指定的CSV文件中读取位置和欧拉角数据，并将欧拉角转换为四元数。
    CSV文件格式应为：x, y, z, yaw, pitch, roll

    参数:
    filename (str): 包含位置和欧拉角数据的CSV文件路径

    返回:
    list[tuple[float]]: 一个列表，每个元素是一个包含位置和四元数的元组 (x, y, z, qx, qy, qz, qw)
    """
    converted_data = []
    
    with open(filename, 'r', newline='', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)  # 跳过表头（如果有）

        for row in reader:
            try:
                x, y, z, yaw, pitch, roll = map(float, row[:6])
                quaternion = angles_to_quaternion(yaw, pitch, roll)
                converted_data.append((x, y, z, quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
            except ValueError:
                rospy.logwarn(f"Invalid data in CSV file at line: {row}")

    return converted_data


def send_goals_to_move_base(converted_data):
    """
    从给定的包含位置和四元数数据的列表中读取目标位置和方向数据，并以设定的频率发布到'/send_mark_goal'主题。

    参数:
    converted_data (list[tuple[float]]): 包含位置和四元数数据的列表
    """
    rospy.init_node('send_goal_node', anonymous=True)

    goal_pub = rospy.Publisher('/send_mark_goal', PoseStamped, queue_size=1)
    rate = rospy.Rate(1)  # 设置发布频率为每秒1次

    for i, (x, y, z, qx, qy, qz, qw) in enumerate(converted_data):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        print(x)
        rate.sleep()
        rospy.sleep(0.1)
       
        goal_pub.publish(goal_pose)
        print(y)
        
        rospy.sleep(0.1)


if __name__ == '__main__':
    target_data_file = "/home/robot/Ros_Test/Test_07/src/nav_demo/scripts/data.csv"  # 替换为实际文件路径
    converted_data = read_and_convert_csv(target_data_file)
    send_goals_to_move_base(converted_data)