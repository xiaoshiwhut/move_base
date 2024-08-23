/*********************************************************************
 *
 *  BSD 3-Clause License
 *
 *  Copyright (c) 2021, dengpw
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1 Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2 Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   3 Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Author:  dengpw
 *********************************************************************/
#include <iostream>
#include "planner_core.h"
#include <tf/transform_datatypes.h>
#include <ros/node_handle.h>
#include "astar.h"
#include "hybrid_astar.h"

PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)//注册为类插件的声明

namespace hybrid_astar_planner {

HybridAStarPlanner::HybridAStarPlanner():
    initialized_(false),costmap(NULL),resolution(1.0) {
  std::cout << "creating the hybrid Astar planner" << std::endl;
}


void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}


void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *_costmap, std::string frame_id) {
  if(!initialized_) {
    ROS_INFO("initializing the hybrid Astar planner");
    // 订阅global_costmap的内容，以便获取参数
    ros::NodeHandle nh("~/global_costmap");
    ros::NodeHandle nh2("~/");
    ros::NodeHandle private_nh("~/" + name);
    nh2.param("use_hybrid_astar", use_hybrid_astar, true);
    if(use_hybrid_astar) {
      ROS_INFO("Using hybrid_astar mode!");
    } else {
      ROS_INFO("Using Astar mode!");
    }
    costmap = _costmap;
    frame_id_ = frame_id;
    std::cout << frame_id << std::endl;
    //  初始化发布路径的主题
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    path_vehicles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
    make_plan_srv_ = private_nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);
  }
  initialized_ = true;
}//end of constructor function HybridAStarPlanner

HybridAStarPlanner::~HybridAStarPlanner() {

}//end of deconstructor function HybridAStarPlanner


bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, 
    nav_msgs::GetPlan::Response& resp) {
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;
  return true;
}



// 辅助函数：从四元数中获取方向角
double HybridAStarPlanner::getThetaFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)).getRPY(roll, pitch, yaw);
    return yaw;
}


// 新增一个方法来生成节点的移动方向向量
std::vector<int> HybridAStarPlanner::generateNodeDirectionVector(std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<int> directionVector; // 用于存储节点移动方向的向量

    // 遍历路径中的所有节点，除了最后一个
    for (size_t i = 0; i < plan.size() - 1; ++i) {
        // 获取当前节点和下一个节点的位置和方向
        const geometry_msgs::PoseStamped& current_pose = plan[i];
        const geometry_msgs::PoseStamped& next_pose = plan[i + 1];

        // 计算从当前节点到下一节点的相对位置向量
        double dx = next_pose.pose.position.x - current_pose.pose.position.x;
        double dy = next_pose.pose.position.y - current_pose.pose.position.y;

        // 计算移动方向的角度
        double theta_move = atan2(dy, dx);

        // 获取当前节点的方向角
        double theta_current = getThetaFromQuaternion(current_pose.pose.orientation);

        // 将角度限制在 [-π, π] 区间内
        while (theta_move > M_PI) theta_move -= 2 * M_PI;
        while (theta_move < -M_PI) theta_move += 2 * M_PI;

        // 计算方向差值
        double delta_theta = theta_current - theta_move;
        while (delta_theta > M_PI) delta_theta -= 2 * M_PI;
        while (delta_theta < -M_PI) delta_theta += 2 * M_PI;

        // 如果方向差值小于 π/2，则认为是向前移动
        if (std::abs(delta_theta) < M_PI / 2) {
            directionVector.push_back(1); // 当前节点向下一节点移动是向前，添加1到向量
        } else {
            directionVector.push_back(-1); // 当前节点向下一节点移动是向后，添加0到向量
        }
        plan[i].pose.position.z = directionVector.back();
        

       
    }
    // plan.back().pose.position.z = 1;

    return directionVector;
}



bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal, 
    std::vector<geometry_msgs::PoseStamped>& plan) {
  // std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
  // std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
  Expander* _planner;

  // ROS_INFO("the resolution of cost map: %f ",costmap->getResolution());
  if (use_hybrid_astar) {  
    _planner = new hybridAstar(frame_id_,costmap);
  }
  else {
    _planner = new astar(frame_id_,costmap);
  }

  //检查设定的目标点参数是否合规
  if(!(checkStartPose(start) && checkgoalPose(goal))) {
    ROS_WARN("Failed to create a global plan!");
    return false;
  }
  plan.clear();
  //正式将参数传入规划器中
  if(!_planner->calculatePath(start, goal , costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), plan, path_vehicles_pub_, pathNodes)) {
    return false;
  }
  
  //参数后期处理，发布到RViz上进行可视化
  clearPathNodes();

  // 生成节点顺序向量
  std::vector<int> directionVector = generateNodeDirectionVector(plan);
 

  // 打印移动方向向量
  for (int direction : directionVector) {
      if (direction == 1) {
          ROS_INFO("Node Direction: %d, should move forward", direction);
      } else {
          ROS_INFO("Node Direction: %d, should move backward", direction);
      }
  }
  
  //path只能发布2D的节点
  
  publishPlan(plan);
  publishPathNodes(plan);
  return true;
}//end of makeplan

bool HybridAStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start) {
    unsigned int startx,starty;
  if (costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty)) {
    return true;
  }
  ROS_WARN("The Start pose is out of the map!");
  return false;
}//end of checkStartPose

bool HybridAStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal) {
  unsigned int goalx,goaly;
  if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly)) {
    if (costmap->getCost( goalx, goaly ) > 252) {
      // std::cout << costmap->getCost(goalx, goaly) << std::endl;
      ROS_WARN("The Goal pose is out of the map! %d",costmap->getCost(goalx, goaly));
      ROS_WARN("The Goal pose is occupied , please reset the goal!");
      return false;
    }
    return true;
  }
  return false;
}//end of checkgoalPose

}//namespace hybrid_astar_planner

