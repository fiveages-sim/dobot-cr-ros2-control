//
// Dobot Gripper Interface - Abstract interface for Dobot gripper implementations
//
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

// 前向声明
class CRCommanderRos2;

namespace dobot_ros2_control
{
    /**
     * @brief 夹爪接口类
     *
     * 定义夹爪的标准接口，支持不同的夹爪实现（Changingtek, Jodell等）
     */
    class DobotGripperInterface
    {
    public:
        virtual ~DobotGripperInterface() = default;

        /**
         * @brief 初始化夹爪
         * @param commander Dobot 通信器
         * @return 是否初始化成功
         */
        virtual bool initialize(
            std::shared_ptr<CRCommanderRos2> commander
        ) = 0;

        /**
         * @brief 读取夹爪状态
         * 注意：此方法会直接更新内部状态，可通过 getPositionPtr() 获取
         * @return 是否成功读取
         */
        virtual bool readStatus() = 0;

        /**
         * @brief 写入夹爪命令
         * @param position 夹爪位置命令（0.0=闭合, 1.0=打开）
         * @return 是否成功发送命令
         */
        virtual bool writeCommand(double position) = 0;

        /**
         * @brief 检查是否应该读取状态（用于频率控制）
         * @return 是否应该读取
         */
        virtual bool shouldRead() = 0;

        /**
         * @brief 获取夹爪关节名称
         * @return 夹爪关节名称
         */
        virtual const std::string& getJointName() const = 0;

        /**
         * @brief 获取位置指针（用于 hardware interface）
         * @return 位置指针
         */
        virtual double* getPositionPtr() = 0;

        /**
         * @brief 获取位置命令指针（用于 hardware interface）
         * @return 位置命令指针
         */
        virtual double* getPositionCommandPtr() = 0;
    };
} // namespace dobot_ros2_control
