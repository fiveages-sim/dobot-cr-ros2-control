// Copyright 2024 FiveAges Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dobot_ros2_control/grippers/changingtek_gripper.h"
#include "dobot_ros2_control/command.h"
#include <cmath>

using namespace gripper_hardware_common;
using ModbusCfg = ModbusConfig::Changingtek90C;

namespace dobot_ros2_control
{
    ChangingtekGripper::ChangingtekGripper(
        const std::string& joint_name,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        int read_frequency_divider
    )
        : joint_name_(joint_name)
          , logger_(logger)
          , clock_(clock)
          , read_controller_(read_frequency_divider)
    {
    }

    bool ChangingtekGripper::initialize(
        std::shared_ptr<CRCommanderRos2> commander
    )
    {
        if (!commander || !commander->isConnected())
        {
            RCLCPP_ERROR(logger_, "Dobot commander not connected");
            return false;
        }

        commander_ = commander;

        // 初始化 Modbus
        if (!initializeModbus(commander))
        {
            RCLCPP_ERROR(logger_, "Failed to initialize Modbus for gripper!");
            return false;
        }

        // 读取夹爪当前位置
        if (readStatus())
        {
            position_command_ = position_;
            last_command_ = position_;
            RCLCPP_INFO(logger_, "Changingtek gripper initialized, initial position: %.3f", position_);
        }
        else
        {
            RCLCPP_WARN(logger_, "Failed to read initial gripper position, using 0.0");
            position_ = 0.0;
            position_command_ = 0.0;
            last_command_ = 0.0;
        }

        RCLCPP_INFO(logger_, "✅ Changingtek gripper initialized (integrated control mode)");
        return true;
    }

    bool ChangingtekGripper::initializeModbus(std::shared_ptr<CRCommanderRos2> commander)
    {
        if (!commander || !commander->isConnected())
        {
            RCLCPP_ERROR(logger_, "Cannot initialize Modbus: not connected");
            return false;
        }

        RCLCPP_INFO(logger_, "Initializing Modbus for Changingtek gripper control...");

        // 步骤1: 关闭所有已存在的Modbus连接（0-4）
        for (int i = 0; i < 5; i++)
        {
            commander->modbusClose(i);
            // 不检查返回值，因为连接可能本来就不存在
        }

        // 步骤2: 创建新的Modbus RTU连接
        // 参数：slave_id=1, baud=115200, parity='N', data_bit=8, stop_bit=1
        if (!commander->modbusRTUCreate(1, 115200, "N", 8, 1))
        {
            RCLCPP_ERROR(logger_, "Failed to create Modbus RTU connection");
            return false;
        }

        RCLCPP_INFO(logger_, "✅ Modbus initialized successfully");
        return true;
    }

    bool ChangingtekGripper::readStatus()
    {
        if (!commander_ || !commander_->isConnected())
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Cannot read gripper status: commander not connected"
            );
            return false;
        }

        // 读取寄存器 0x060D (1549)，2个寄存器
        std::string result;
        if (!commander_->getHoldRegs(0, ModbusCfg::FEEDBACK_REG_ADDR, 2, "U16", result))
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to read gripper feedback register (0x%04X)", ModbusCfg::FEEDBACK_REG_ADDR
            );
            return false;
        }

        // 解析结果: "{val1,val2}"
        // 去掉大括号
        if (result.size() < 3)
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Invalid gripper feedback format: %s (too short)", result.c_str()
            );
            return false;
        }

        std::string data = result.substr(1, result.size() - 2); // 移除 { }

        // 分割字符串
        size_t comma_pos = data.find(',');
        if (comma_pos == std::string::npos)
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Invalid gripper feedback format: %s (no comma)", result.c_str()
            );
            return false;
        }

        try
        {
            int val1 = std::stoi(data.substr(0, comma_pos));
            int val2 = std::stoi(data.substr(comma_pos + 1));

            // 计算位置：(high << 16) + low
            uint32_t modbus_pos = ((uint32_t)val1 << 16) | val2;

            // 使用通用库的位置转换器，直接更新内部状态
            position_ = PositionConverter::Changingtek90::modbusToNormalized(modbus_pos);

            RCLCPP_DEBUG_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Gripper position updated: %.3f (modbus: %u)", position_, modbus_pos
            );

            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to parse gripper feedback: %s, error: %s", result.c_str(), e.what()
            );
            return false;
        }
    }

    bool ChangingtekGripper::writeCommand(double position)
    {
        if (!commander_ || !commander_->isConnected())
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Cannot send gripper command: commander not connected"
            );
            return false;
        }

        // 使用通用库的命令变化检测器
        if (!CommandChangeDetector::hasChanged(position, last_command_))
        {
            return true; // 命令未变化，无需发送，但返回 true 表示正常
        }

        // 使用通用库的位置转换器
        uint16_t modbus_value = PositionConverter::Changingtek90::normalizedToModbus(position);

        // 发送三个Modbus寄存器写入命令（异步发送，不等待响应）
        // 寄存器258 (0x0102): 控制模式 = 0
        if (!commander_->setHoldRegs(0, ModbusCfg::POS_REG_ADDR, 1, "{0}", "U16"))
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to write gripper control mode register (0x%04X)", ModbusCfg::POS_REG_ADDR
            );
            return false;
        }

        // 寄存器259 (0x0103): 目标位置
        char val_buf[32];
        snprintf(val_buf, sizeof(val_buf), "{%u}", modbus_value);
        if (!commander_->setHoldRegs(0, ModbusCfg::POS_REG_ADDR + 1, 1, val_buf, "U16"))
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to write gripper position register (0x%04X)", ModbusCfg::POS_REG_ADDR + 1
            );
            return false;
        }

        // 寄存器264 (0x0108): 执行命令 = 1
        if (!commander_->setHoldRegs(0, ModbusCfg::TRIGGER_REG_ADDR, 1, "{1}", "U16"))
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to write gripper trigger register (0x%04X)", ModbusCfg::TRIGGER_REG_ADDR
            );
            return false;
        }

        last_command_ = position;
        RCLCPP_DEBUG(logger_, "Gripper command sent: %.3f -> %u", position, modbus_value);
        return true;
    }

    bool ChangingtekGripper::shouldRead()
    {
        return read_controller_.shouldRead();
    }
} // namespace dobot_ros2_control
