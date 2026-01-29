/**
 * @file keyboard_control_node.cpp
 * @author Nesc (raymond059@mails.gdut.edu.cn)
 * @brief 键盘控制器节点
 * @version 0.1
 * @date 2025-12-06
 * 
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <cmath>
#include <algorithm>

// 该变量保存终端设置
struct termios original_termios;
bool termios_saved = false;

void restore_terminal()
{
    if (termios_saved) 
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
    }
}

/**
 * @brief 信号处理函数，捕捉终端信号以恢复终端设置
 * 
 * @param signum 信号编号
 */
void signal_handler(int signum) 
{
    restore_terminal();
    ros::shutdown();
    exit(signum);
}

/**
 * @brief 主函数，键盘控制节点入口
 * 
 * 流程：
 * 1. 初始化ROS节点和发布者
 * 2. 保存并修改终端设置以实现非阻塞键盘输入
 * 3. 主循环中读取键盘输入，根据按键更新目标速度
 * 4. 实现动态超时逻辑以调整响应速度
 * 5. 发布速度指令
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // 保存原始终端设置
    tcgetattr(STDIN_FILENO, &original_termios);
    termios_saved = true;
    
    // 注册清理函数
    atexit(restore_terminal);
    // 捕捉 SIGINT 信号以恢复终端设置
    signal(SIGINT, signal_handler);

    // 修改终端设置为非阻塞模式
    struct termios new_termios = original_termios;
    new_termios.c_lflag &= ~ICANON;
    new_termios.c_lflag &= ~ECHO;
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    // 控制参数
    double base_speed = 3.0;
    double turbo_speed = 5.0;
    double angular_speed = 7.0;

    std::cout << "Keyboard Control Node Started" << std::endl;
    std::cout << "Use w/s for x-axis, a/d for y-axis." << std::endl;
    std::cout << "Use q/e/z/c for diagonal movement." << std::endl;
    std::cout << "Press f to toggle rotation." << std::endl;
    std::cout << "Hold Shift for turbo speed." << std::endl;
    std::cout << "Space to brake immediately." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;

    // 初始化速度消息
    geometry_msgs::Twist current_twist;
    current_twist.linear.x = 0; current_twist.linear.y = 0; current_twist.linear.z = 0;
    current_twist.angular.x = 0; current_twist.angular.y = 0; current_twist.angular.z = 0;

    // 目标速度变量
    double target_x = 0;
    double target_y = 0;
    double target_z = 0;

    // 旋转状态标志
    bool is_rotating = false;
    ros::Time last_f_time(0);

    ros::Time last_input_time = ros::Time::now();
    
    // 动态超时逻辑
    double initial_timeout = 0.6; 
    double repeat_timeout = 0.15;
    double current_timeout = initial_timeout;

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        char c;
        while (read(STDIN_FILENO, &c, 1) > 0) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_input_time).toSec();
            last_input_time = now;

            // Dynamic timeout adjustment
            if (dt < 0.1) 
            {
                current_timeout = repeat_timeout;
            } 
            else if (dt > 0.5) 
            {
                current_timeout = initial_timeout;
            }
            
            // 检查 Ctrl+C 退出
            if (c == 3) 
            {
                restore_terminal();
                ros::shutdown();
                return 0;
            }

            // 处理按键输入
            switch(c) 
            {
                case 'w': target_x = base_speed; target_y = 0; last_input_time = now; break;
                case 'W': target_x = turbo_speed; target_y = 0; last_input_time = now; break;
                case 's': target_x = -base_speed; target_y = 0; last_input_time = now; break;
                case 'S': target_x = -turbo_speed; target_y = 0; last_input_time = now; break;
                
                case 'a': target_x = 0; target_y = base_speed; last_input_time = now; break;
                case 'A': target_x = 0; target_y = turbo_speed; last_input_time = now; break;
                case 'd': target_x = 0; target_y = -base_speed; last_input_time = now; break;
                case 'D': target_x = 0; target_y = -turbo_speed; last_input_time = now; break;

                case 'q': target_x = base_speed; target_y = base_speed; last_input_time = now; break;
                case 'Q': target_x = turbo_speed; target_y = turbo_speed; last_input_time = now; break;
                case 'e': target_x = base_speed; target_y = -base_speed; last_input_time = now; break;
                case 'E': target_x = turbo_speed; target_y = -turbo_speed; last_input_time = now; break;

                case 'f': 
                case 'F':
                    if ((now - last_f_time).toSec() > 0.5) 
                    {
                        is_rotating = !is_rotating;
                        last_f_time = now;
                        std::cout << "Rotation: " << (is_rotating ? "ON" : "OFF") << std::endl;
                    }
                    break;
                
                case 'z': target_x = -base_speed; target_y = base_speed; last_input_time = now; break;
                case 'Z': target_x = -turbo_speed; target_y = turbo_speed; last_input_time = now; break;
                case 'c': target_x = -base_speed; target_y = -base_speed; last_input_time = now; break;
                case 'C': target_x = -turbo_speed; target_y = -turbo_speed; last_input_time = now; break;
                    
                case ' ': // Brake
                    target_x = 0;
                    target_y = 0;
                    is_rotating = false;
                    break;
            }
        }

        // 检查超时
        if ((ros::Time::now() - last_input_time).toSec() > current_timeout) 
        {
            target_x = 0;
            target_y = 0;
            current_timeout = initial_timeout;
        }

        // 设置角速度目标值
        target_z = is_rotating ? angular_speed : 0.0;

        // 速度平滑滤波器 (限制加速度)
        double dt = 1.0 / 50.0;
        double accel_limit = 1.0; // 1 m/s^2
        double step = accel_limit * dt;

        // X轴速度滤波
        if (current_twist.linear.x < target_x)
            current_twist.linear.x = std::min(target_x, current_twist.linear.x + step);
        else if (current_twist.linear.x > target_x)
            current_twist.linear.x = std::max(target_x, current_twist.linear.x - step);

        // Y轴速度滤波
        if (current_twist.linear.y < target_y)
            current_twist.linear.y = std::min(target_y, current_twist.linear.y + step);
        else if (current_twist.linear.y > target_y)
            current_twist.linear.y = std::max(target_y, current_twist.linear.y - step);

        current_twist.angular.z = target_z;
        pub.publish(current_twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
