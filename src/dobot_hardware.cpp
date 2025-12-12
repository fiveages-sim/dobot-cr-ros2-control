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

#include "dobot_ros2_control/dobot_hardware.h"
#include "dobot_ros2_control/grippers/changingtek_gripper.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <map>
#include <unordered_map>

namespace dobot_ros2_control
{
    hardware_interface::CallbackReturn DobotHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 初始化夹爪
        gripper_.reset();
        has_gripper_ = false;
        gripper_joint_index_ = -1;

        // 初始化力传感器标志
        has_ft_sensor_ = false;

        // 解析 URDF 配置的关节（使用基类成员变量 info_）
        int joint_index = 0;
        for (const auto& joint : info_.joints)
        {
            // 检查关节名称中是否包含 gripper 或 hand
            std::string joint_name_lower = joint.name;
            std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                           joint_name_lower.begin(), ::tolower);

            if (joint_name_lower.find("gripper") != std::string::npos ||
                joint_name_lower.find("hand") != std::string::npos)
            {
                // 这是夹爪关节
                has_gripper_ = true;
                gripper_joint_index_ = joint_index;
                RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %s (index %d)",
                            joint.name.c_str(), gripper_joint_index_);
            }
            else
            {
                // 这是机械臂关节
                joint_names_.push_back(joint.name);
            }
            joint_index++;
        }

        // 初始化关节数据存储（根据实际配置的关节数量）
        int arm_joint_count = static_cast<int>(joint_names_.size());
        joint_positions_.resize(arm_joint_count, 0.0);
        joint_velocities_.resize(arm_joint_count, 0.0);
        joint_efforts_.resize(arm_joint_count, 0.0);
        joint_position_commands_.resize(arm_joint_count, 0.0);

        RCLCPP_INFO(get_node()->get_logger(), "Found %d arm joints in configuration", arm_joint_count);

        // 检测力传感器配置
        hardware_interface::ComponentInfo ft_sensor_info;
        has_ft_sensor_ = findSensorByName("ft_sensor", ft_sensor_info);
        if (has_ft_sensor_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "Found ft_sensor: %s", ft_sensor_info.name.c_str());
        }

        // 读取配置参数的辅助函数（使用基类成员变量 info_）
        const auto get_hardware_parameter = [this
            ](const std::string& parameter_name, const std::string& default_value)
        {
            if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
            {
                return it->second;
            }
            return default_value;
        };

        // 读取配置参数
        robot_ip_ = get_hardware_parameter("robot_ip", "192.168.5.38");
        servo_time_ = std::stod(get_hardware_parameter("servo_time", "0.03"));
        aheadtime_ = std::stod(get_hardware_parameter("aheadtime", "20.0"));
        gain_ = std::stod(get_hardware_parameter("gain", "500.0"));
        speed_factor_ = std::stoi(get_hardware_parameter("speed_factor", "5"));
        verbose_ = get_hardware_parameter("verbose", "false") == "true";

        // 如果检测到夹爪，创建夹爪对象
        if (has_gripper_)
        {
            std::string gripper_joint_name;
            for (const auto& joint : info_.joints)
            {
                std::string joint_name_lower = joint.name;
                std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                               joint_name_lower.begin(), ::tolower);
                if (joint_name_lower.find("gripper") != std::string::npos ||
                    joint_name_lower.find("hand") != std::string::npos)
                {
                    gripper_joint_name = joint.name;
                    break;
                }
            }

            std::string gripper_type = get_hardware_parameter("gripper_type", "changingtek");
            int read_frequency_divider = std::stoi(get_hardware_parameter("gripper_read_frequency_divider", "4"));

            std::unordered_map<std::string, std::string> gripper_params;
            gripper_params["read_frequency_divider"] = std::to_string(read_frequency_divider);

            gripper_ = createGripper(gripper_type, gripper_joint_name, get_node()->get_logger(), gripper_params);
            if (!gripper_)
            {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to create gripper, continuing without gripper");
                has_gripper_ = false;
            }
        }

        // 初始化力传感器数据（用于 hardware interface）
        ft_sensor_force_x_ = 0.0;
        ft_sensor_force_y_ = 0.0;
        ft_sensor_force_z_ = 0.0;
        ft_sensor_torque_x_ = 0.0;
        ft_sensor_torque_y_ = 0.0;
        ft_sensor_torque_z_ = 0.0;

        // 初始化统计变量
        write_count_ = 0;
        last_write_stat_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_node()->get_logger(), "==============================================");
        RCLCPP_INFO(get_node()->get_logger(), "DobotHardware initialized successfully");
        RCLCPP_INFO(get_node()->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  ServoJ Time: %.3f s", servo_time_);
        RCLCPP_INFO(get_node()->get_logger(), "  Aheadtime: %.1f", aheadtime_);
        RCLCPP_INFO(get_node()->get_logger(), "  Gain: %.1f", gain_);
        RCLCPP_INFO(get_node()->get_logger(), "  Speed Factor: %d%%", speed_factor_);
        RCLCPP_INFO(get_node()->get_logger(), "  Verbose: %s", verbose_ ? "true" : "false");
        RCLCPP_INFO(get_node()->get_logger(), "  Arm Joints: %zu", joint_names_.size());
        if (has_gripper_ && gripper_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "  Gripper: Yes (%s)", gripper_->getJointName().c_str());
        }
        if (has_ft_sensor_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "  FT Sensor: Yes");
        }
        RCLCPP_INFO(get_node()->get_logger(), "==============================================");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    DobotHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating DobotHardware...");

        try
        {
            // 创建TCP连接到机器人（传递 verbose 参数）
            commander_ = std::make_shared<CRCommanderRos2>(robot_ip_, verbose_);
            commander_->init();

            RCLCPP_INFO(get_node()->get_logger(), "TCP connection initialized to %s", robot_ip_.c_str());

            // 等待TCP连接建立
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for TCP connection...");

            auto start_time = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::seconds(10);

            while (!commander_->isConnected())
            {
                auto current_time = std::chrono::steady_clock::now();
                if (current_time - start_time > timeout)
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Timeout waiting for TCP connection!");
                    return hardware_interface::CallbackReturn::ERROR;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            RCLCPP_INFO(get_node()->get_logger(), "✅ TCP connection established!");

            // 设置全局速度
            RCLCPP_INFO(get_node()->get_logger(), "Setting global speed factor to %d%%...", speed_factor_);
            if (!commander_->setSpeedFactor(speed_factor_))
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to set speed factor!");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 如果配置了夹爪，初始化夹爪
            if (has_gripper_ && gripper_)
            {
                if (!gripper_->initialize(commander_))
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize gripper!");
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            // 读取当前关节位置并初始化命令（机械臂关节）
            double current_joints[6];
            commander_->getCurrentJointStatus(current_joints);

            // 无需锁：ros2_control 保证 read/write 顺序执行，无并发访问
            const size_t arm_joints = std::min(joint_names_.size(), static_cast<size_t>(6)); // 最多6个关节
            for (size_t i = 0; i < arm_joints; ++i)
            {
                joint_positions_[i] = current_joints[i];
                joint_position_commands_[i] = current_joints[i];
            }

            RCLCPP_INFO(get_node()->get_logger(),
                        "Initial %zu arm joint positions read from robot", arm_joints);

            RCLCPP_INFO(get_node()->get_logger(), "✅ DobotHardware activated and ready!");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate DobotHardware: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DobotHardware::on_deactivate(
        const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating DobotHardware...");

        // 断开TCP连接
        commander_.reset();

        RCLCPP_INFO(get_node()->get_logger(), "✅ DobotHardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> DobotHardware::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        // 为机械臂关节导出状态接口：位置、速度、力矩
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
        }

        // 如果配置了夹爪，导出夹爪状态接口（只有位置）
        if (has_gripper_ && gripper_)
        {
            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    gripper_->getJointName(), hardware_interface::HW_IF_POSITION, gripper_->getPositionPtr()));
        }

        // 导出六维力传感器状态接口
        if (has_ft_sensor_)
        {
            hardware_interface::ComponentInfo ft_sensor_info;
            if (!findSensorByName("ft_sensor", ft_sensor_info))
            {
                RCLCPP_WARN(get_node()->get_logger(),
                            "ft_sensor flag is true but sensor not found in configuration");
                return state_interfaces;
            }
            // 创建接口名称到值的映射
            std::map<std::string, double*> interface_map = {
                {"force.x", &ft_sensor_force_x_},
                {"force.y", &ft_sensor_force_y_},
                {"force.z", &ft_sensor_force_z_},
                {"torque.x", &ft_sensor_torque_x_},
                {"torque.y", &ft_sensor_torque_y_},
                {"torque.z", &ft_sensor_torque_z_}
            };

            // 按照配置中定义的顺序导出接口
            for (size_t i = 0; i < ft_sensor_info.state_interfaces.size() && i < interface_map.size(); ++i)
            {
                const std::string& interface_name = ft_sensor_info.state_interfaces[i].name;
                auto it = interface_map.find(interface_name);
                if (it != interface_map.end())
                {
                    state_interfaces.push_back(
                        std::make_shared<hardware_interface::StateInterface>(
                            ft_sensor_info.name, interface_name, it->second));
                }
                else
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Unknown ft_sensor interface: %s", interface_name.c_str());
                }
            }

            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu ft_sensor state interfaces", ft_sensor_info.state_interfaces.size());
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Exported %zu state interfaces (%zu arm joints%s%s)",
                    state_interfaces.size(), joint_names_.size(),
                    has_gripper_ ? " + 1 gripper" : "",
                    has_ft_sensor_ ? " + ft_sensor" : "");

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> DobotHardware::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        // 为机械臂关节导出命令接口：位置命令
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
        }

        // 如果配置了夹爪，导出夹爪命令接口
        if (has_gripper_ && gripper_)
        {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    gripper_->getJointName(), hardware_interface::HW_IF_POSITION, gripper_->getPositionCommandPtr()));

            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu command interfaces (%zu arm joints + 1 gripper)",
                        command_interfaces.size(), joint_names_.size());
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu command interfaces for %zu arm joints",
                        command_interfaces.size(), joint_names_.size());
        }

        return command_interfaces;
    }

    hardware_interface::return_type DobotHardware::read(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/)
    {
        if (!commander_ || !commander_->isConnected())
        {
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            // 从TCP连接读取当前关节状态
            double current_joints[6];
            commander_->getCurrentJointStatus(current_joints);

            // 更新关节位置（无需锁：ros2_control 框架保证顺序执行）
            size_t arm_joints = std::min(joint_names_.size(), size_t(6)); // 最多6个关节
            for (size_t i = 0; i < arm_joints; ++i)
            {
                joint_positions_[i] = current_joints[i];
            }

            // 获取实时数据（包含速度和力矩信息）
            auto real_time_data = commander_->getRealData();
            if (real_time_data)
            {
                for (size_t i = 0; i < arm_joints; ++i)
                {
                    joint_velocities_[i] = real_time_data->qd_actual[i] * M_PI / 180.0; // 转换为弧度/秒
                    joint_efforts_[i] = real_time_data->m_actual[i]; // 力矩
                }

                // SixForceValue通常包含：Fx, Fy, Fz, Mx, My, Mz
                // 前3个是力（Force），后3个是力矩（Torque）
                ft_sensor_force_x_ = real_time_data->SixForceValue[0];
                ft_sensor_force_y_ = real_time_data->SixForceValue[1];
                ft_sensor_force_z_ = real_time_data->SixForceValue[2];
                ft_sensor_torque_x_ = real_time_data->SixForceValue[3];
                ft_sensor_torque_y_ = real_time_data->SixForceValue[4];
                ft_sensor_torque_z_ = real_time_data->SixForceValue[5];
            }

            // 夹爪状态读取：定期读取（降低读取频率以减少Modbus负载）
            if (has_gripper_ && gripper_)
            {
                if (gripper_->shouldRead())
                {
                    if (!gripper_->readStatus())
                    {
                        RCLCPP_ERROR_THROTTLE(
                            get_node()->get_logger(),
                            *get_node()->get_clock(),
                            2000,
                            "Failed to read gripper state"
                        );
                    }
                    // readStatus 内部已经更新了 position_，无需额外处理
                }
            }

            // 定期打印调试信息（仅 verbose 模式）
            if (verbose_)
            {
                // 只打印前几个关节的位置
                RCLCPP_DEBUG_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    1000,
                    "Joint positions updated (%zu joints)",
                    joint_names_.size()
                );
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Error reading joint states: %s", e.what()
            );
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DobotHardware::write(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/)
    {
        if (!commander_ || !commander_->isConnected())
        {
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            // 获取关节命令（弧度）（无需锁：ros2_control 框架保证顺序执行）
            double joint_cmd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            size_t arm_joints = std::min(joint_names_.size(), static_cast<size_t>(6));
            for (size_t i = 0; i < arm_joints; ++i)
            {
                joint_cmd[i] = joint_position_commands_[i];
            }

            // 通过ServoJ发送关节命令（异步发送，不等待响应）
            // 注意：ServoJ 始终发送6个关节值，未配置的关节发送0
            bool success = commander_->servoJ(joint_cmd, servo_time_, aheadtime_, gain_);

            if (!success)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    1000,
                    "Failed to send ServoJ command"
                );
                return hardware_interface::return_type::ERROR;
            }

            // 夹爪控制：检测命令变化并发送
            // 注意：writeCommand 内部已经处理了命令变化检测和错误日志
            if (has_gripper_ && gripper_)
            {
                double target = *gripper_->getPositionCommandPtr();
                gripper_->writeCommand(target);
                // writeCommand 内部会记录错误（如果有），这里不需要额外处理
            }

            // 统计写入频率（仅在 verbose 模式下显示）
            if (verbose_)
            {
                write_count_++;
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - last_write_stat_time_).count();

                if (elapsed >= 1000)
                {
                    // 每1秒输出一次
                    double frequency = write_count_ * 1000.0 / elapsed;
                    RCLCPP_INFO(get_node()->get_logger(),
                                "[Write] Frequency: %.2f Hz (%d cmds in %ld ms)",
                                frequency, write_count_, elapsed);

                    // 重置计数器
                    write_count_ = 0;
                    last_write_stat_time_ = current_time;
                }
            }

            // 定期打印命令值（仅 verbose 模式）
            if (verbose_)
            {
                RCLCPP_DEBUG_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    1000,
                    "Joint commands sent (%zu joints)",
                    joint_names_.size()
                );
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Error writing joint commands: %s", e.what()
            );
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    std::unique_ptr<DobotGripperInterface> DobotHardware::createGripper(
        const std::string& gripper_type,
        const std::string& joint_name,
        rclcpp::Logger logger,
        const std::unordered_map<std::string, std::string>& params
    )
    {
        std::unique_ptr<DobotGripperInterface> gripper;

        if (gripper_type == "changingtek" || gripper_type == "changingtek90")
        {
            int read_frequency_divider = 4;
            if (auto it = params.find("read_frequency_divider"); it != params.end())
            {
                read_frequency_divider = std::stoi(it->second);
            }
            gripper = std::make_unique<ChangingtekGripper>(joint_name, logger, get_node()->get_clock(),
                                                           read_frequency_divider);
            RCLCPP_INFO(logger, "Created Changingtek gripper: %s", joint_name.c_str());
        }
        else
        {
            RCLCPP_WARN(logger, "Unknown gripper type: %s, supported types: changingtek", gripper_type.c_str());
            return nullptr;
        }

        return gripper;
    }

    bool DobotHardware::findSensorByName(const std::string& sensor_name,
                                         hardware_interface::ComponentInfo& sensor_info)
    {
        for (const auto& sensor : info_.sensors)
        {
            if (sensor.name.find(sensor_name) != std::string::npos)
            {
                sensor_info = sensor;
                return true;
            }
        }
        return false;
    }
} // namespace dobot_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(dobot_ros2_control::DobotHardware, hardware_interface::SystemInterface)
