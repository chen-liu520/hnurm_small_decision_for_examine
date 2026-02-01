#include "GameStart.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace hnurm_small_decision{

    GameStart::GameStart(const std::string &condition_name, const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_game_start_(false),
          stop_listener_(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        RCLCPP_INFO(node_->get_logger(), "[GameStart] 等待按回车键开始游戏...");

        // 启动后台线程监听键盘
        listener_thread_ = std::thread(&GameStart::keyboardListener, this);
    }

    GameStart::~GameStart()
    {
        stop_listener_ = true;
        if (listener_thread_.joinable())
        {
            listener_thread_.join();
        }
    }

    BT::NodeStatus GameStart::tick()
    {
        if (is_game_start_.load())
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void GameStart::keyboardListener()
    {
        // 保存原始终端设置
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;

        // 设置为非规范模式（无需等待回车），但先不关闭回显
        new_tio.c_lflag &= ~(ICANON);
        new_tio.c_cc[VMIN] = 0;  // 非阻塞读取
        new_tio.c_cc[VTIME] = 0; // 无超时
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        while (!stop_listener_.load() && !is_game_start_.load())
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0)
            {
                if (c == '\n')
                { // 检测到回车键
                    is_game_start_.store(true);
                    RCLCPP_INFO(node_->get_logger(), "[GameStart] 收到回车键，游戏开始！");
                }
            }
            // 小延时避免忙等
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_small_decision::GameStart>("GameStart");
}