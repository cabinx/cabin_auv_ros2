#ifndef T4_CONTROLLER_HPP_
#define T4_CONTROLLER_HPP_
//#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "cabin_teleop/t4_button_mapping.h"
#include "sensor_msgs/msg/joy.hpp"
//#include "cabin_msgs/msg/net_load.hpp"
//#include "cabin_msgs/msg/switch_state.hpp"
//#include "cabin_msgs/msg/reset_controls.hpp"

#include "cabin_msgs/msg/net_load.hpp"
#include "cabin_msgs/msg/switch_state.hpp"
#include "cabin_msgs/msg/reset_controls.hpp"


class T4Controller : public rclcpp::Node{
    public:
        T4Controller();
        virtual ~T4Controller() = default;

    private:
        // joystick param
        double joy_force[3];                  //The force input along the x, y, z axis
        double joy_moment[3];                 //The moment input around the x, y, axis
        bool joy_switch_state;
        double max_forward_thrust;
        double max_backward_thrust;
        double max_forward_thrust_ud;
        double max_backward_thrust_ud;
        double current_axes_factor[4];

        cabin_msgs::msg::NetLoad output_netLoad;
        cabin_msgs::msg::SwitchState output_switch_state;
        cabin_msgs::msg::ResetControls output_reset_state;
        
        void joystick_param_init();        
        // subscribe the joystick signal
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        void JoyCallback(const sensor_msgs::msg::Joy msg);
        // publish the control signal
        rclcpp::Publisher<cabin_msgs::msg::NetLoad>::SharedPtr joy_netLoad_pub;
        rclcpp::TimerBase::SharedPtr timer_netLoad;
        void timer_callback_netLoad();
        // lock state and reset state
        rclcpp::Publisher<cabin_msgs::msg::SwitchState>::SharedPtr joy_switch_pub;
        rclcpp::Publisher<cabin_msgs::msg::ResetControls>::SharedPtr joy_reset_pub;
        
        size_t count_;
};
#endif