//
// Changingtek Gripper - Changingtek gripper implementation for Dobot robots
//
#pragma once

#include "dobot_ros2_control/grippers/dobot_gripper_interface.h"
#include "gripper_hardware_common/ChangingtekGripper.h"
#include <memory>
#include <string>

// 前向声明
class CRCommanderRos2;

namespace dobot_ros2_control
{
    /**
     * @brief Changingtek 夹爪实现类（用于 Dobot 机器人）
     *
     * 通过 Dobot 机器人的 Modbus 接口控制 Changingtek 夹爪
     */
    class ChangingtekGripper : public DobotGripperInterface
    {
    public:
        /**
         * @brief 构造函数
         * @param joint_name 夹爪关节名称
         * @param logger ROS2日志记录器
         * @param clock ROS2时钟（用于限流日志）
         * @param read_frequency_divider 读取频率除数（默认4）
         */
        explicit ChangingtekGripper(
            const std::string& joint_name,
            rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock,
            int read_frequency_divider = 4
        );

        /**
         * @brief 析构函数
         */
        ~ChangingtekGripper() override = default;

        /**
         * @brief 初始化夹爪
         * @param commander Dobot 通信器
         * @return 是否初始化成功
         */
        bool initialize(
            std::shared_ptr<CRCommanderRos2> commander
        ) override;

        /**
         * @brief 读取夹爪状态
         * 注意：此方法会直接更新内部的 position_ 成员变量
         * @return 是否成功读取
         */
        bool readStatus() override;

        /**
         * @brief 写入夹爪命令
         * @param position 夹爪位置命令（0.0=闭合, 1.0=打开）
         * @return 是否成功发送命令
         */
        bool writeCommand(double position) override;

        /**
         * @brief 检查是否应该读取状态（用于频率控制）
         * @return 是否应该读取
         */
        bool shouldRead() override;

        /**
         * @brief 获取夹爪关节名称
         * @return 夹爪关节名称
         */
        const std::string& getJointName() const override { return joint_name_; }

        /**
         * @brief 获取位置指针（用于 hardware interface）
         * @return 位置指针
         */
        double* getPositionPtr() override { return &position_; }

        /**
         * @brief 获取位置命令指针（用于 hardware interface）
         * @return 位置命令指针
         */
        double* getPositionCommandPtr() override { return &position_command_; }

    private:
        // 初始化 Modbus
        bool initializeModbus(std::shared_ptr<CRCommanderRos2> commander);

        // 成员变量
        std::string joint_name_; // 夹爪关节名称
        std::shared_ptr<CRCommanderRos2> commander_ = nullptr; // Dobot 通信器
        rclcpp::Logger logger_; // ROS2日志记录器
        rclcpp::Clock::SharedPtr clock_; // ROS2时钟（用于限流日志）

        // 夹爪状态
        double position_ = 0.0; // 夹爪位置（0.0=闭合, 1.0=打开）
        double position_command_ = 0.0; // 夹爪位置命令（0.0-1.0）
        double last_command_ = 0.0; // 上次发送的夹爪命令

        // 工具类
        gripper_hardware_common::ReadFrequencyController read_controller_; // 读取频率控制器
    };
} // namespace dobot_ros2_control
