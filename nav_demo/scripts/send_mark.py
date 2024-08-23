#!/usr/bin/env python3
# encoding: utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import *
import sys, select, termios, tty
import time
import threading
from nav_demo.msg import myMsg
sys.path.append('/home/v2t/move_base/src')

'''
    定义动作

'''
global y_position,x_position
y_position = 0
x_position = 0
def execute_bucket_action(bucket_action):
    pub1 = rospy.Publisher('/cmd_bucket', Float32, queue_size=10)
    pub2 = rospy.Publisher('/cmd_arm', Float32, queue_size=10)
    bucket_msg = Float32()
    arm_msg =Float32()
    # 动作控制逻辑
    if bucket_action == 1:  # 示例落铲动作，实际逻辑待定
        rospy.loginfo("落铲动作执行逻辑")
        # 铲斗控制模拟
        for _ in range(10):
            arm_msg.data = 1 if y_position > 600 else -1 if y_position < 520 else 0  # 控制铲斗角度
            bucket_msg.data = 0
            pub1.publish(bucket_msg)
            pub2.publish(arm_msg)
            rospy.sleep(0.6)  # 控制频率
        for _ in range(10):  # 假设循环模拟调整过程
            arm_msg.data = 0  # 假设控制臂的线性速度
            if x_position < 520:
                arm_msg.data = 1  # 向上移动
            elif x_position > 600:
                arm_msg.data = -1  # 向下移动
            else:
                arm_msg.data = 0  # 保持位置
            pub2.publish(arm_msg)
            pub1.publish(bucket_msg)
            rospy.sleep(0.6)  # 控制频率，模拟调整时间
        rospy.loginfo("落铲动作完成")
        
    elif bucket_action == 2:  # 抬铲后抖铲动作
        rospy.loginfo("抬铲后抖铲动作开始")
        # 铲臂控制模拟
        for _ in range(15):  # 假设循环模拟调整过程
            arm_msg.data = 0  # 假设控制臂的线性速度
            if y_position < 520:
                bucket_msg.data = 1  # 向上移动
            elif y_position > 550:
                bucket_msg.data = -1  # 向下移动
            else:
                bucket_msg.data = 0  # 保持位置
            pub1.publish(bucket_msg)
            pub2.publish(arm_msg)
            rospy.sleep(0.6)  # 控制频率，模拟调整时间
        # 铲斗控制模拟
        for _ in range(10):
            arm_msg.data = 1 if x_position > 550 else -1 if x_position < 520 else 0  # 控制铲斗角度
            bucket_msg.data = 0
            pub1.publish(bucket_msg)
            pub2.publish(arm_msg)
            rospy.sleep(0.6)  # 控制频率
        rospy.loginfo("抬铲后抖铲动作完成")
    else:
        rospy.logwarn("未知动作类型")




def callback(data):
        """
        回调函数，更新铲斗高度和角度。
        """
        global y_position,x_position
        y_position = data.y_position
        x_position = data.x_position
        
        # rospy.loginfo("Bucket Height: %s, Bucket Angle: %s", self.bucket_height, self.bucket_angle)
        # self.check_and_perform_action(self.default_action)






#到达目标点成功或失败的回调函数，输入参数：[3：成功， 其它：失败](4：ABORTED)
def pose_callback(msg):
    global try_again, index, try_again, index

    
    if msg.status.status == 3 and count>0 :  #成功到达任意目标点，前往下一目标点
        try_again = 1 #允许再次尝试前往尚未抵达的该目标点

                   
        #count表示当前目标点计数，index表示已完成的目标点计数
        if index == count:                   #当index等于count时，表示所有目标点完成，重新开始巡航
            print ('Reach the target point '+str(index-1)+':')
            print('x:'+str(markerArray.markers[index-1].pose.position.x)+
                ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                ', w:'+str(markerArray.markers[index-1].pose.orientation.w))   

            if count>1: print ('Complete instructions!') #只有一个目标点不算巡航
            index = 0
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
            
            
            
            execute_bucket_action(2)
           
            # index += 1 #下一次要发布的目标点序号
            # goal_pub.publish(pose)



            

        elif index < count:                   #当index小于count时，表示还未完成所有目标点，目标巡航未完成
            print ('Reach the target point '+str(index-1)+':')    
            print('x:'+str(markerArray.markers[index-1].pose.position.x)+
                ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
            

            execute_bucket_action(1)
        
            index += 1 #下一次要发布的目标点序号
            goal_pub.publish(pose)
            
            
            # index += 1 #下一次要发布的目标点序号
        
    elif count>0: #未抵达设定的目标点    
        rospy.logwarn('Can not reach the target point '+str(index-1)+':'+'\r\n'+
                      'x:'+str(markerArray.markers[index-1].pose.position.x)+
                    ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                    ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                    ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

        #如果未尝试过前往尚未抵达的目标点，则尝试前往尚未抵达的目标点
        if try_again == 1:
            rospy.logwarn('trying reach the target point '+str(index-1)+' again!'+'\r\n'+
                          'x:'+str(markerArray.markers[index-1].pose.position.x)+
                        ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                        ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                        ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index - 1].pose.position.x           
            pose.pose.position.y = markerArray.markers[index - 1].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index-1].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index-1].pose.orientation.w
            goal_pub.publish(pose)
            try_again = 0 #不允许再次尝试前往尚未抵达的该目标点

        #如果已经尝试过前往尚未抵达的目标点，则前往下一个目标点
        elif index < len(markerArray.markers):      #若还未完成目标点
            rospy.logwarn('try reach the target point '+str(index-1)+' failed! reach next point:'+'\r\n'+
                          'x:'+str(markerArray.markers[index-1].pose.position.x)+
                        ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                        ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                        ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

            if index==count: index=0 #如果下一个目标点序号为count，说明当前目标点为最后一个目标点，下一个目标点序号应该设置为0
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x      
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w

            execute_bucket_action(1)
            index += 1 #下一次要发布的目标点序号
            goal_pub.publish(pose)
            

            # index += 1 #下一次要发布的目标点序号
            try_again = 1 #允许再次尝试前往尚未抵达的该目标点

    
#rviz内NavGoal标记按下的回调函数，输入参数：按下的位置[x, y, z=0]
def navGoal_callback(msg):           
    global index, count

    print('Add a new target point '+str(count)+':')
    print('x:'+str(msg.pose.position.x)+
        ', y:'+str(msg.pose.position.y)+
        ', z:'+str(msg.pose.orientation.z)+
        ', w:'+str(msg.pose.orientation.w)) 

    marker = Marker()      #创建marker对象
    marker.header.frame_id = 'map' #以哪一个TF坐标为原点
    marker.type = marker.ARROW#TEXT_VIEW_FACING #一直面向屏幕的字符格式
    marker.action = marker.ADD #添加marker
    marker.scale.x = 0.5 #marker大小
    marker.scale.y = 0.05 #marker大小
    marker.scale.z = 0.05 #marker大小，对于字符只有z起作用
    marker.color.a = 1 #字符透明度
    marker.color.r = 1 #字符颜色R(红色)通道
    marker.color.g = 0 #字符颜色G(绿色)通道
    marker.color.b = 0 #字符颜色B(蓝色)通道
    marker.pose.position.x = msg.pose.position.x #字符位置
    marker.pose.position.y = msg.pose.position.y #字符位置
    marker.pose.position.z = 0.1#msg.position.z #字符位置
    marker.pose.orientation.z = msg.pose.orientation.z #字符位置
    marker.pose.orientation.w = msg.pose.orientation.w #字符位置
    markerArray.markers.append(marker) #添加元素进数组

    marker_number = Marker()      #创建marker对象
    marker_number.header.frame_id = 'map' #以哪一个TF坐标为原点
    marker_number.type = marker_number.TEXT_VIEW_FACING #一直面向屏幕的字符格式
    marker_number.action = marker_number.ADD #添加marker
    marker_number.scale.x = 0.5 #marker大小
    marker_number.scale.y = 0.5 #marker大小
    marker_number.scale.z = 0.5 #marker大小，对于字符只有z起作用
    marker_number.color.a = 1 #字符透明度
    marker_number.color.r = 1 #字符颜色R(红色)通道
    marker_number.color.g = 0 #字符颜色G(绿色)通道
    marker_number.color.b = 0 #字符颜色B(蓝色)通道
    marker_number.pose.position.x = msg.pose.position.x #字符位置
    marker_number.pose.position.y = msg.pose.position.y #字符位置
    marker_number.pose.position.z = 0.1#msg.position.z #字符位置
    marker_number.pose.orientation.z = msg.pose.orientation.z #字符位置
    marker_number.pose.orientation.w = msg.pose.orientation.w #字符位置
    marker_number.text = str(count) #字符内容
    markerArray_number.markers.append(marker_number) #添加元素进数组

    #markers的id不能一样，否则rviz只会识别最后一个元素
    id = 0
    for m in markerArray.markers:    #遍历marker分别给id赋值
        m.id = id
        id += 1

    for m in markerArray_number.markers:    #遍历marker分别给id赋值
        m.id = id
        id += 1

    mark_pub.publish(markerArray) #发布markerArray，rviz订阅并进行可视化
    mark_pub.publish(markerArray_number) #发布markerArray，rviz订阅并进行可视化

    #第一次添加marker时直接发布目标点
    if count == 0:
        pose = PoseStamped() #创建目标点对象
        pose.header.frame_id = 'map' #以哪一个TF坐标为原点
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.pose.position.x #目标点位置
        pose.pose.position.y = msg.pose.position.y #目标点位置
        pose.pose.orientation.z = marker.pose.orientation.z
        pose.pose.orientation.w = marker.pose.orientation.w
        goal_pub.publish(pose)
        index += 1 #下一次要发布的目标点序号

    count += 1 #有几个目标点

 


#获取键值函数
def getKey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    try:
        # termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
        # tty.setraw(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
    return key

def send_mark():
    global markerArray, markerArray_number, count, index, try_again, mark_pub, goal_pub
    markerArray = MarkerArray() #目标点标记数组
    markerArray_number = MarkerArray() #目标点标记数组
    count = 0  #count 有几个目标点
    index = 0
    try_again = 1
    sendflagPublisher = rospy.Publisher('/send_flag', Int8, queue_size =1)
    rospy.init_node('path_point_demo') #初始化节点
    mark_pub    = rospy.Publisher('/path_point', MarkerArray, queue_size = 100) #用于发布目标点标记
    navGoal_sub = rospy.Subscriber('/send_mark_goal', PoseStamped, navGoal_callback) #订阅rviz内标记按下的位置
    # click_sub   = rospy.Subscriber('/clicked_point', PointStamped, click_callback) #订阅rviz内标记按下的位置
    goal_pub    = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1) #用于发布目标点
    send_flag=Int8()
    send_flag.data=1
    sendflagPublisher.publish(send_flag)
    rospy.sleep(1.)
    sendflagPublisher.publish(send_flag)
    rospy.loginfo('a=%d',send_flag.data)
    rospy.loginfo("多点导航程序启动")
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, pose_callback) #用于订阅是否到达目标点状态
    action_sub = rospy.Subscriber("/nav_demo", myMsg, callback)
    #sub1 = rospy.Subscriber("/cmd_bucket", myMsg, callback)
    #sub2 = rospy.Subscriber("/cmd_arm", myMsg, callback)
    while not rospy.is_shutdown():
        key = getKey() #获取键值
        if(key=='c'): #键值为c是清空目标点
            count = 0
            index = 0
            try_again = 1

            markerArray = MarkerArray() 
            marker = Marker()
            marker.header.frame_id = 'map' 
            marker.type = marker.TEXT_VIEW_FACING 
            marker.action = marker.DELETEALL 
            marker.text = '' 
            markerArray.markers.append(marker) 

            for m in markerArray_number.markers:    
                m.action = marker.DELETEALL

            mark_pub.publish(markerArray) 
            mark_pub.publish(markerArray_number) 
            markerArray = MarkerArray() 
            markerArray_number = MarkerArray() 

        elif (key == '\x03'): #ctrl+c退出
            break
def breakkey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)



'''
    读取csv文件数据及发布多点导航目标点代码
'''




#可直接由python3运行，用rosrun运行有问题


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化
    rospy.on_shutdown(breakkey)#退出前执行键值初始化
    send_mark()  #正常导航2D nva Goal话题/move_base_simple/goal  多点导航/send_mark_goal111
    
    # print("当前活跃线程的数量", threading.active_count())
    # print("将当前所有线程的具体信息展示出来", threading.enumerate())
    # print("当前的线程的信息展示", threading.current_thread())
    # print('{} is running '.format(threading.current_thread().name))
