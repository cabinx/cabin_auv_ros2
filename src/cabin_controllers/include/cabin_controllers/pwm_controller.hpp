#ifndef PWM_CONTROLLER_HPP
#define PWM_CONTROLLER_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <unistd.h>

#include "cabin_msgs/msg/pwm_stamped.hpp"
#include "cabin_msgs/msg/thrust_stamped.hpp"
#include "cabin_msgs/msg/switch_state.hpp"
#include "cabin_msgs/msg/reset_controls.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std;

class PWMController : public rclcpp::Node{
    public:
        PWMController();
        virtual ~PWMController() = default;

    private:
        std::string ns;               //namespace 
        YAML::Node properties;
        YAML::Node pwmProperties;
        string propertiesFile;
        string pwmFile;

        int thrusterType[8];
        float startup_config[2][4],primary_config[2][4];     //Slopes and y-intercepts
        float critical_thrusts[2][2];                        //Minimum and startup thrusts

        cabin_msgs::msg::PwmStamped pwm_msg;
        bool dead, silent, reset_pwm;
        rclcpp::Time last_alive_time;
        rclcpp::Duration *a;
        double alive_time_out;

        template<typename T>
        void LoadParam(std::string param, T &var);
        void LoadThrusterProperties();

        rclcpp::Subscription<cabin_msgs::msg::ThrustStamped>::SharedPtr cmd_sub;
        rclcpp::Subscription<cabin_msgs::msg::SwitchState>::SharedPtr kill_sub;
        rclcpp::Subscription<cabin_msgs::msg::ResetControls>::SharedPtr reset_sub;
        rclcpp::Publisher<cabin_msgs::msg::PwmStamped>::SharedPtr pwm_pub;
        rclcpp::TimerBase::SharedPtr timer_pwm_pub;
        void timer_callback_pwm_pub();

        void ThrustCB(cabin_msgs::msg::ThrustStamped thrust);
        void SwitchCB(cabin_msgs::msg::SwitchState state);
        void ResetController(cabin_msgs::msg::ResetControls reset_msg);

        int Thrust2pwm(double raw_force, int type, std::string name);

        size_t count_;
};
#endif