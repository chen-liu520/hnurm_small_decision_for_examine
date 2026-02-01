#include "PubRobotStatus.hpp"
#include "string"

namespace hnurm_small_decision {
    using std::placeholders::_1;

    PubRobotStatus::PubRobotStatus(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(xml_tag_name, conf)
    {
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
        /**************************回调组 start**************************************/
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        executor_thread_ = std::thread([this]()
                                       { callback_group_executor_.spin(); });
        RCLCPP_INFO(node_->get_logger(), "PubRobotStatus::回调组多线程启动");
        /**************************回调组 end**************************************/


        /**************************参数相关 start**********************************/
        cmd_vel_topic_ = node_->declare_parameter("cmd_vel_topic", "/cmd_vel");

        global_position_topic_ = node_->declare_parameter("global_position_topic", "/global_position");

        cmd_vel_remap_topic_ = node_->declare_parameter("cmd_vel_remap_topic", "/cmd_vel_remap");
        
        node_->declare_parameter("cruise_goals", std::vector<std::string>());

        /**************************参数相关 end**********************************/
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;



        /**************************订阅者 start**********************************/
        global_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
            global_position_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::global_pose_callback, this, _1),
            sub_option);

        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::remap_cmd_vel_callback, this, _1),
            sub_option);
        /**************************订阅者 end**********************************/

        cmd_vel_remap_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_remap_topic_, 10);

        RCLCPP_INFO(node_->get_logger(), "PubRobotStatus::构造函数完成");
    }

    PubRobotStatus::~PubRobotStatus()
    {
        if (executor_thread_.joinable())
        {
            executor_thread_.join(); // 这个析构函数的主要作用是确保线程资源的正确回收
        }
        // 如果不调用 join()，线程可能在对象销毁后继续运行，导致以下问题：
        // 访问已释放的对象内存，引发未定义行为
        // 线程资源（如栈空间）无法被操作系统回收，造成内存泄漏
    }

    void PubRobotStatus::initialization(){
        auto cruise_goals_from_param_str_ = node_->get_parameter("cruise_goals").as_string_array();

        for (const auto &point : cruise_goals_from_param_str_)
        {
            GlobalPose gp;
            size_t comma_pos = point.find(',');
            gp.pose_x = std::stod(point.substr(0, comma_pos));
            gp.pose_y = std::stod(point.substr(comma_pos + 1));
            cruise_goals_from_param_.push_back(gp);
        }
        for (const auto &goal : cruise_goals_from_param_)
        {
            RCLCPP_INFO(node_->get_logger(), "Loaded goal: (%.2f, %.2f)",
                        goal.pose_x, goal.pose_y);
        }

        FillGoalsDeque(cruise_goals_from_param_); // 填充两个队列

        if (!goal_poses_deque_for_check_.empty())
        {
            current_goal_x_y = goal_poses_deque_for_check_.front();
            current_goal_pose_ = goal_poses_deque_for_nav2_.front();
        }

        is_global_init_ = true;

        // 调试信息
        RCLCPP_INFO(node_->get_logger(), "PubRobotStatus::初始化完成，目标点队列大小: %zu",
                    goal_poses_deque_for_check_.size());
    }

    BT::NodeStatus PubRobotStatus::tick()
    {
        
        if (!is_global_init_)
        {
            initialization();
        }

        bool is_reached = false;
        {
            std::lock_guard<std::mutex> lock(current_x_y_mutex_);
            is_reached = CheckIsReached(current_x_y, current_goal_x_y, 0.5);
        }
        
        if (goal_poses_deque_for_check_.empty() || goal_poses_deque_for_nav2_.empty())
        {
            setOutput("run_state", false);
            return BT::NodeStatus::FAILURE; 
        }
        if(is_reached)
        {
            // 到达目标点，弹出队列
            goal_poses_deque_for_nav2_.pop_front();
            goal_poses_deque_for_check_.pop_front();

            // 调试信息
            RCLCPP_INFO(node_->get_logger(), "到达目标点: (%.2f, %.2f), 队列去除该点",
                        current_goal_x_y.pose_x, current_goal_x_y.pose_y);
            RCLCPP_INFO(node_->get_logger(), "剩余目标点数量: %zu",
                        goal_poses_deque_for_check_.size());

            if (!goal_poses_deque_for_check_.empty())
            {
                current_goal_x_y = goal_poses_deque_for_check_.front();
                current_goal_pose_ = goal_poses_deque_for_nav2_.front();
            }else{
                setOutput("run_state", false);
                // 调试信息
                RCLCPP_INFO(node_->get_logger(), "所有目标点已到达，导航结束");
            }
        }
        is_navigating_ = !goal_poses_deque_for_check_.empty();
        setOutput("cruise_goal", current_goal_pose_);
        setOutput("run_state", is_navigating_);
        setOutput("goal_poses_deque", goal_poses_deque_for_nav2_);
        setOutput("current_controller", "Omni");
        return BT::NodeStatus::SUCCESS;
    }


    /***********************************************工具函数 start *********************************************************/
    /* @name : GlobalPose2PoseStamped
     *  @brief : 将全局坐标点转换为PoseStamped消息
     *  @param : global_pose 全局坐标点
     *  @return : pose_stamped PoseStamped消息
     */
    geometry_msgs::msg::PoseStamped PubRobotStatus::GlobalPose2PoseStamped(GlobalPose global_pose){
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = node_->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = global_pose.pose_x;
        pose_stamped.pose.position.y = global_pose.pose_y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        return pose_stamped;
    }

    /*  @name : FillGoalsDeque
     *  @brief : 填充goal_poses_deque_for_nav2_和goal_poses_deque_for_check_
     *  @param : cruise_goals_from_param 全局坐标点向量
     *  @return : None
     */
    void PubRobotStatus::FillGoalsDeque(std::vector<GlobalPose> cruise_goals_from_param){
        for (const auto &goal : cruise_goals_from_param)
        {
            goal_poses_deque_for_nav2_.push_back(GlobalPose2PoseStamped(goal));
            goal_poses_deque_for_check_.push_back(goal);
        }
    }

    /*  @name : CheckIsReached
     *  @brief : 检查是否到达导航点
     *  @param : current_pose 当前机器人位姿(来自订阅的导航话题)，current_goal_pose 当前导航点（来自队列top（））, threshold 到达阈值
     *  @return : bool 是否到达导航点
     */
    bool PubRobotStatus::CheckIsReached(GlobalPose current_pose, GlobalPose current_goal_pose, double threshold){
        double distance = std::sqrt(std::pow(current_pose.pose_x - current_goal_pose.pose_x, 2) +
                                    std::pow(current_pose.pose_y - current_goal_pose.pose_y, 2));
        return distance <= threshold;
    }

    /* @name: calculateAverage
     *  @brief：用于计算多边形点集的平均坐标，作为机器人当前位置的估计值。将定位系统发布的 footprint（多边形）转换为机器人的中心点坐标
     *  @param：msg geometry_msgs::msg::PolygonStamped msg - 包含多边形点集的 ROS 消息
     */
    PubRobotStatus::GlobalPose PubRobotStatus::calculateAverage(geometry_msgs::msg::PolygonStamped &msg)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        GlobalPose cgp_;
        for (auto const &p : msg.polygon.points)
        {
            x_sum += p.x;
            y_sum += p.y;
        }
        cgp_.pose_x = x_sum / msg.polygon.points.size();
        cgp_.pose_y = y_sum / msg.polygon.points.size();
        return cgp_;
    }
    /***********************************************工具函数 end *********************************************************/

    /**************************回调函数 start*********************************
     * @name : global_pose_callback
     * @brief : 全局定位回调函数,填充现在机器人坐标
     * @param : msg 全局定位消息
     * @return : None
     */
    void PubRobotStatus::global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg){
        std::lock_guard<std::mutex> lock(current_x_y_mutex_);
        current_x_y = calculateAverage(*msg);
    }

    /*
     * @name : remap_cmd_vel_callback
     * @brief : 速度命令回调函数,【当前方案是：减速，可以变化】
     * @param : msg 速度命令消息
     * @return : None
     */
    void PubRobotStatus::remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        // 减速
        geometry_msgs::msg::Twist vel_pub_;
        vel_pub_ = *msg; 
        // vel_pub_.linear.x *= 0.5;
        // vel_pub_.linear.y *= 0.5;
        // cmd_vel_remap_pub_->publish(vel_pub_);
    }

    /**************************回调函数 end*********************************/
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_small_decision::PubRobotStatus>("PubRobotStatus");
}