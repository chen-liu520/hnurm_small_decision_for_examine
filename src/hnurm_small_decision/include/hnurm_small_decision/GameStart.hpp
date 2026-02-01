#ifndef GAMESTART_HPP
#define GAMESTART_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <behaviortree_cpp_v3/condition_node.h>
#include <atomic>
#include <thread>

/*
终端焦点：必须保证运行节点的终端有焦点，否则捕获不到按键
ROS2 launch：如果用 launch 文件启动，确保 output="screen"：
<node pkg="hnurm_small_decision" exec="decision_node" output="screen"/>
退出问题：按了回车之后，后台线程会自动退出，不会一直占用资源
多次 tick：一旦返回 SUCCESS，后续 tick 都会保持 SUCCESS（因为 is_game_start_ 不会重置）
*/
namespace hnurm_small_decision
{

    class GameStart : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for hnurm_small_decision::GameStart
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        GameStart(
            const std::string &condition_name, // 节点在行为树中的名字
            const BT::NodeConfiguration &conf  // BT节点配置
        );

        ~GameStart();

        GameStart() = delete; // 删除默认构造函数

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override; // 重写tick函数，用于执行节点的逻辑

        /**
         * @brief Creates list of BT ports，用于表示节点的输入输出端口
         * @return BT::PortsList Containing node-specific ports，当前返回空列表，表示该节点没有输入或输出端口
         */
        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        /**
         * @brief 监听和处理键盘输入，判断游戏是否开始
         */
        void keyboardListener(); // 后台线程函数

        // ROS 2 节点实例的共享指针
        rclcpp::Node::SharedPtr node_;
        std::atomic<bool> is_game_start_; // 线程安全的标志位
        std::atomic<bool> stop_listener_; // 停止监听标志
        std::thread listener_thread_;     // 键盘监听线程

    };

} // namespace hnurm_small_behavior_tree

#endif