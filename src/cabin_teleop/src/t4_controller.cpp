#include "cabin_teleop/t4_controller.hpp"

T4Controller:: T4Controller()
 : rclcpp::Node("joystick_sub", "cabin_auv"), count_(0){
    using std::placeholders::_1;
    // joystick param init
    joystick_param_init();
    //subscribe the joystick signal and update the param
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
         "joy", rclcpp::SensorDataQoS(), std::bind(&T4Controller::JoyCallback, this, _1));

    //load the current param and pub NetLoad msg
    joy_netLoad_pub = this->create_publisher<cabin_msgs::msg::NetLoad>("command/netLoad", 1);
    timer_netLoad = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&T4Controller::timer_callback_netLoad, this));

    //the lock for safety
    joy_switch_pub = this->create_publisher<cabin_msgs::msg::SwitchState>("state/switches", 1);
    //reset the input
    joy_reset_pub = this->create_publisher<cabin_msgs::msg::ResetControls>("command/reset", 1);
    
 }

void T4Controller::joystick_param_init(){
        joy_switch_state = false;
        max_forward_thrust = 33.0;
        max_backward_thrust = 25.0;
        max_forward_thrust_ud = 60.0;
        max_backward_thrust_ud = 60.0;
}

void T4Controller::JoyCallback(const sensor_msgs::msg::Joy msg){
    //Increase/decrease 5.0 eachtime
    if(1 == msg.axes[AXES_CROSS_UD]){
        joy_force[0] += 5.0;
        RCLCPP_INFO(get_logger(), "current joystick input thrust is %f!!!", joy_force[0]);
    }
    else if(-1 == msg.axes[AXES_CROSS_UD]){
        joy_force[0] -= 5.0;
        RCLCPP_INFO(get_logger(), "current joystick input thrust is %f!!!", joy_force[0]);
    }
    else{
        joy_force[0] += 0.0;
    }

    if(1 == msg.axes[AXES_CROSS_LR]){
        joy_moment[2] += 5.0;
        RCLCPP_INFO(get_logger(), "current joystick input moment is %f!!!", joy_moment[2]);
    }
    else if(-1 == msg.axes[AXES_CROSS_LR]){
        joy_moment[2] -= 5.0;
        RCLCPP_INFO(get_logger(), "current joystick input moment is %f!!!", joy_moment[2]);
    }
    else{
        joy_moment[2] += 0.0;
    }

    if(msg.axes[AXES_STICK_LEFT_UD]){
        current_axes_factor[0] = msg.axes[AXES_STICK_LEFT_UD];
        if(current_axes_factor[0] > 0){
            joy_force[0] = current_axes_factor[0] * max_forward_thrust;
        }
        else{
            joy_force[0] = current_axes_factor[0] * max_backward_thrust;
        }
    }

    if(msg.axes[AXES_STICK_LEFT_LR]){
        current_axes_factor[1] = msg.axes[AXES_STICK_LEFT_LR];
        if(current_axes_factor[1] > 0){
            joy_moment[2] = current_axes_factor[1] * max_forward_thrust;
        }
        else{
            joy_moment[2] = current_axes_factor[1] * max_backward_thrust;
        }
    }

    if(msg.axes[AXES_STICK_RIGHT_UD]){
        current_axes_factor[2] = msg.axes[AXES_STICK_RIGHT_UD];
        if(current_axes_factor[2] < 0){
            joy_force[2] = current_axes_factor[2] * max_forward_thrust_ud;
        }
        else{
            joy_force[2] = current_axes_factor[2] * max_backward_thrust_ud;
        }
    }

    // the lock button for safety
    if(1 == msg.buttons[BUTTON_SHAPE_A]){
        joy_switch_state = !joy_switch_state;
        rclcpp::Time now = this->now();
        output_switch_state.header.stamp = now;
        output_switch_state.kill = joy_switch_state;
        joy_switch_pub->publish(output_switch_state);
        if(joy_switch_state){
            RCLCPP_INFO(get_logger(), "current joystick lock has opened!!!");
        }
        else{
            RCLCPP_INFO(get_logger(), "current joystick lock has closed!!!");
        }
    }
    // reset the input
    if( 1 == msg.buttons[BUTTON_SHAPE_B]){
        output_reset_state.reset_pwm = false;
        joy_reset_pub->publish(output_reset_state);
        for(int i = 0; i < 3; i++){
            joy_force[i] = 0.0;
            joy_moment[i] = 0.0;
        }
        RCLCPP_INFO(get_logger(), "joystick input has been reset!!!");
    }
}


void T4Controller::timer_callback_netLoad(){
    //current timestamp
    rclcpp::Time now = this->now();
    output_netLoad.header.stamp = now;
    //Move forward
    output_netLoad.force.x = joy_force[0];
    output_netLoad.force.y = joy_force[1];
    output_netLoad.force.z = joy_force[2];
    //Rotate around z axis (yaw)
    output_netLoad.moment.x = joy_moment[0];
    output_netLoad.moment.y = joy_moment[1];
    output_netLoad.moment.z = joy_moment[2];

    joy_netLoad_pub->publish(output_netLoad);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor joy_sub_executor;
    auto joy_sub_node = std::make_shared<T4Controller>();
    joy_sub_executor.add_node(joy_sub_node);
    joy_sub_executor.spin();
    joy_sub_executor.remove_node(joy_sub_node);
    return 0;
}