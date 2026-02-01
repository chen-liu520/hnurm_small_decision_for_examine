#ifndef PUB_ROBOT_STATUS_HPP_
#define PUB_ROBOT_STATUS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/bool.hpp>
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include "deque"
#include "mutex"
#include <atomic>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>


namespace hnurm_small_decision {

    class PubRobotStatus : public BT::SyncActionNode
    {
        public:
            PubRobotStatus(
                const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);
            ~PubRobotStatus() override;
            static BT::PortsList providedPorts()
            {
                // BT::OutputPort<bool>("黑板变量名", "黑板变量描述"),
                // setOutput("黑板变量名"， 变量值)
                return {
                    BT::OutputPort<geometry_msgs::msg::PoseStamped>("cruise_goal", "cruise goal pose"),
                    BT::OutputPort<bool>("run_state", "true:is running, false:is not running"),

                    BT::OutputPort<std::deque<geometry_msgs::msg::PoseStamped>>("goal_poses_deque", "goal_poses_deque"),
                    BT::OutputPort<std::string>("current_controller", "current_controller"),
                };
            }
            rclcpp::Node::SharedPtr node_;
            // 全局坐标点
            struct GlobalPose
            {
                float pose_x;
                float pose_y;
            };
            // 导航点队列（给nav2使用）
            std::deque<geometry_msgs::msg::PoseStamped> goal_poses_deque_for_nav2_;

            // 导航点队列（给自己判断使用）
            std::deque<GlobalPose> goal_poses_deque_for_check_;
            
            // 从参数服务器获取的巡航目标点【vector】
            std::vector<GlobalPose> cruise_goals_from_param_;

            // 全局初始化标志位
            bool is_global_init_ = false;

            // 是否正在导航，（"run_state"）
            bool is_navigating_ = false;

            // 当前导航点【坐标】
            GlobalPose current_goal_x_y;
            // 当前导航点【位姿】
            geometry_msgs::msg::PoseStamped current_goal_pose_;

            // current_x_y 互斥锁
            std::mutex current_x_y_mutex_;

            // 当前机器人【坐标】
            GlobalPose current_x_y;
            // 当前机器人【位姿】
            geometry_msgs::msg::PoseStamped current_pose_;

            // 回调组
            rclcpp::CallbackGroup::SharedPtr callback_group_; 
            rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

            /****************************初始化 start*******************************/
            void initialization();
            /****************************初始化 end*******************************/
        private:
            BT::NodeStatus tick() override;




            /**************************工具函数 start*********************************/
            /* @name : GlobalPose2PoseStamped
            *  @brief : 将全局坐标点转换为PoseStamped消息
            *  @param : global_pose 全局坐标点
            *  @return : pose_stamped PoseStamped消息
            */
            geometry_msgs::msg::PoseStamped GlobalPose2PoseStamped(GlobalPose global_pose);

            /*  @name : FillGoalsDeque
             *  @brief : 填充goal_poses_deque_for_nav2_和goal_poses_deque_for_check_
             *  @param : cruise_goals_from_param 全局坐标点向量
             *  @return : None
             */
            void FillGoalsDeque(std::vector<GlobalPose> cruise_goals_from_param);


            /*  @name : CheckIsReached
             *  @brief : 检查是否到达导航点
             *  @param : current_pose 当前机器人位姿(来自订阅的导航话题)，current_goal_pose 当前导航点（来自队列top（））, threshold 到达阈值
             *  @return : bool 是否到达导航点
             */
            bool CheckIsReached(GlobalPose current_pose, GlobalPose current_goal_pose, double threshold);

            /* @name: calculateAverage
            *  @brief：用于计算多边形点集的平均坐标，作为机器人当前位置的估计值。将定位系统发布的 footprint（多边形）转换为机器人的中心点坐标
            *  @param：msg geometry_msgs::msg::PolygonStamped msg - 包含多边形点集的 ROS 消息
            */
            GlobalPose calculateAverage(geometry_msgs::msg::PolygonStamped& msg);
            /**************************工具函数 end*********************************/





            /***************************订阅和发布 start*********************************/
            // 订阅全局【定位】系统发布的机器人位置
            rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr global_pose_sub_;
            // 订阅速度命令,重映射
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

            std::string cmd_vel_topic_;

            std::string global_position_topic_;


            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_remap_pub_;

            std::string cmd_vel_remap_topic_;
            /***************************订阅和发布 end*********************************/



            /**************************回调函数 start*********************************
             * @name : global_pose_callback
             * @brief : 全局定位回调函数,填充现在机器人坐标
             * @param : msg 全局定位消息
             * @return : None
             */
            void global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

            /*
             * @name : cmd_vel_callback
             * @brief : 速度命令回调函数
             * @param : msg 速度命令消息
             * @return : None
             */
            void remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

             /**************************回调函数 end*********************************/

            std::thread executor_thread_; // 执行器线程，用于处理回调函数
    };
}

#endif
