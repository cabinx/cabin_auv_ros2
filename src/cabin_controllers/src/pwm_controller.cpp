#include "cabin_controllers/pwm_controller.hpp"
//Critical Thrust Indeces
#define MIN_THRUST 0
#define STARTUP_THRUST 1

//Slope and y-intercept Indeces
#define POS_SLOPE 0
#define POS_YINT 1
#define NEG_SLOPE 2
#define NEG_YINT 3

#define MIN_PWM 1230
#define MAX_PWM 1770
#define NEUTRAL_PWM 1500

PWMController::PWMController()
: rclcpp::Node("pwm_controller", "cabin_auv"), count_(0){
    using std::placeholders::_1;
    ns = this->get_namespace();
    
    //Load parameters from .yaml files or launch files
    PWMController::LoadParam<string>("properties_file", propertiesFile);
    if(access(propertiesFile.c_str(), F_OK)){
        RCLCPP_ERROR(this->get_logger(), "%s Properties file \"%s\" does not exist", ns.c_str(), propertiesFile.c_str());
    }
    properties = YAML::LoadFile(propertiesFile);

    PWMController::LoadParam<string>("pwm_file", pwmFile);
    if(access(propertiesFile.c_str(), F_OK)){
        RCLCPP_ERROR(this->get_logger(), "%s Properties file \"%s\" does not exist", ns.c_str(), propertiesFile.c_str());
    }
    pwmProperties = YAML::LoadFile(pwmFile);
    
    PWMController::LoadThrusterProperties();

    //subscriber and publisher
    cmd_sub = this->create_subscription<cabin_msgs::msg::ThrustStamped>(
         "command/thrust", rclcpp::SensorDataQoS(), std::bind(&PWMController::ThrustCB, this, _1));
    kill_sub = this->create_subscription<cabin_msgs::msg::SwitchState>(
         "state/switches", rclcpp::SensorDataQoS(), std::bind(&PWMController::SwitchCB, this, _1));
    reset_sub = this->create_subscription<cabin_msgs::msg::ResetControls>(
         "command/reset", rclcpp::SensorDataQoS(), std::bind(&PWMController::ResetController, this, _1));
    pwm_pub = this->create_publisher<cabin_msgs::msg::PwmStamped>("command/pwm", 1);
    timer_pwm_pub = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&PWMController::timer_callback_pwm_pub, this));
    
    alive_time_out = 2.0;
    last_alive_time = this->now();

    //last_alive_time += rclcpp::Duration(alive_time_out, 0);
    silent = true; //Silent refers to not receiving commands from the control stack
    dead = true;   //Dead refers to the kill switch being pulled
    reset_pwm = false;   //Refers to Controller being reset via reset command

}

//Load paramter from namespace
template<typename T>
void PWMController::LoadParam(std::string param, T &var){
    this->declare_parameter<std::string>(param, "");
    bool param_get = this->get_parameter(param, var);
    if(!param_get){
        RCLCPP_ERROR(this->get_logger(), "PWM Controller Namespace: %s", ns.c_str());
        RCLCPP_ERROR(this->get_logger(), "Critical! Param \"%s/%s\" does not exist or is not accessed correctly . Shutting down.", ns.c_str(), param.c_str());
        rclcpp::shutdown();
    }
}

// Load parameter from properties files
void PWMController::LoadThrusterProperties(){
    //Load thruster types
    int numThrusters = properties["properties"]["thrusters"].size();
    for(int i = 0; i < numThrusters; i++){
        int type = properties["properties"]["thrusters"][i]["type"].as<int>();
        thrusterType[i] = type;
    }

    //There are 3 regions of thrust: deadband, startup, and primary
    //Deadband is a regin where the propeller will not move due to internal friction and inertia
    //Startup is nonlinear, but is approximated as a line (for now)
    //Primary is linear, but has different slope than startup

    //Critical Thrusts
    critical_thrusts[0][MIN_THRUST] = pwmProperties["TYPE0"]["MIN_THRUST"].as<float>();
    critical_thrusts[0][STARTUP_THRUST] = pwmProperties["TYPE0"]["STARTUP_THRUST"].as<float>();
    critical_thrusts[1][MIN_THRUST] = pwmProperties["TYPE1"]["MIN_THRUST"].as<float>();
    critical_thrusts[1][STARTUP_THRUST] = pwmProperties["TYPE1"]["STARTUP_THRUST"].as<float>();

    //Startup config (Linear Fit: y = mx + b)
    startup_config[0][POS_SLOPE] = pwmProperties["TYPE0"]["SU_POS_THRUST"]["SLOPE"].as<float>();
    startup_config[0][POS_YINT] = pwmProperties["TYPE0"]["SU_POS_THRUST"]["YINT"].as<float>();
    startup_config[0][NEG_SLOPE] = pwmProperties["TYPE0"]["SU_NEG_THRUST"]["SLOPE"].as<float>();
    startup_config[0][NEG_YINT] = pwmProperties["TYPE0"]["SU_NEG_THRUST"]["YINT"].as<float>();

    startup_config[1][POS_SLOPE] = pwmProperties["TYPE1"]["SU_POS_THRUST"]["SLOPE"].as<float>();
    startup_config[1][POS_YINT] = pwmProperties["TYPE1"]["SU_POS_THRUST"]["YINT"].as<float>();
    startup_config[1][NEG_SLOPE] = pwmProperties["TYPE1"]["SU_NEG_THRUST"]["SLOPE"].as<float>();
    startup_config[1][NEG_YINT] = pwmProperties["TYPE1"]["SU_NEG_THRUST"]["YINT"].as<float>();

    //Primary Config (Linear Fit: y = mx +b)
    primary_config[0][POS_SLOPE] = pwmProperties["TYPE0"]["POS_THRUST"]["SLOPE"].as<float>();
    primary_config[0][POS_YINT] = pwmProperties["TYPE0"]["POS_THRUST"]["YINT"].as<float>();
    primary_config[0][NEG_SLOPE] = pwmProperties["TYPE0"]["NEG_THRUST"]["SLOPE"].as<float>();
    primary_config[0][NEG_YINT] = pwmProperties["TYPE0"]["NEG_THRUST"]["YINT"].as<float>();

    primary_config[1][POS_SLOPE] = pwmProperties["TYPE1"]["POS_THRUST"]["SLOPE"].as<float>();
    primary_config[1][POS_YINT] = pwmProperties["TYPE1"]["POS_THRUST"]["YINT"].as<float>();
    primary_config[1][NEG_SLOPE] = pwmProperties["TYPE1"]["NEG_THRUST"]["SLOPE"].as<float>();
    primary_config[1][NEG_YINT] = pwmProperties["TYPE1"]["NEG_THRUST"]["YINT"].as<float>();
}

void PWMController::ThrustCB(cabin_msgs::msg::ThrustStamped thrust){
    if(!dead && !reset_pwm){
        pwm_msg.header.stamp = thrust.header.stamp;
        
        pwm_msg.pwm.vector_port_fwd = Thrust2pwm(thrust.force.vector_port_fwd, thrusterType[thrust.force.VPF], "VPF");
        pwm_msg.pwm.vector_stbd_fwd = Thrust2pwm(thrust.force.vector_stbd_fwd, thrusterType[thrust.force.VSF], "VSF");
        pwm_msg.pwm.vector_port_aft = Thrust2pwm(thrust.force.vector_port_aft, thrusterType[thrust.force.VPA], "VPA");
        pwm_msg.pwm.vector_stbd_aft = Thrust2pwm(thrust.force.vector_stbd_aft, thrusterType[thrust.force.VSA], "VSA");

        pwm_msg.pwm.heave_port_fwd = Thrust2pwm(thrust.force.heave_port_fwd, thrusterType[thrust.force.HPF], "HPF");
        pwm_msg.pwm.heave_stbd_fwd = Thrust2pwm(thrust.force.heave_stbd_fwd, thrusterType[thrust.force.HSF], "HSF");
        pwm_msg.pwm.heave_port_aft = Thrust2pwm(thrust.force.heave_port_aft, thrusterType[thrust.force.HPA], "HPA");
        pwm_msg.pwm.heave_stbd_aft = Thrust2pwm(thrust.force.heave_stbd_aft, thrusterType[thrust.force.HSA], "HSA");

        //pwm_pub->publish(pwm_msg);
        last_alive_time = this->now();
        silent = false;
    }
}

void PWMController::timer_callback_pwm_pub(){
    rclcpp::Time now = this->now();
    double quiet_time = now.seconds() - last_alive_time.seconds();
    if(quiet_time >= alive_time_out){
        silent = true;
    }

    if(silent || dead || reset_pwm){
        pwm_msg.pwm.vector_port_fwd = NEUTRAL_PWM;
        pwm_msg.pwm.vector_stbd_fwd = NEUTRAL_PWM;
        pwm_msg.pwm.vector_port_aft = NEUTRAL_PWM;
        pwm_msg.pwm.vector_stbd_aft = NEUTRAL_PWM;

        pwm_msg.pwm.heave_port_fwd = NEUTRAL_PWM;
        pwm_msg.pwm.heave_stbd_fwd = NEUTRAL_PWM;
        pwm_msg.pwm.heave_port_aft = NEUTRAL_PWM;
        pwm_msg.pwm.heave_stbd_aft = NEUTRAL_PWM;
        
        reset_pwm = false;
    }

    pwm_msg.header.stamp = now;
    pwm_pub->publish(pwm_msg);
}

int PWMController::Thrust2pwm(double raw_force, int type, std::string name){
    int pwm = NEUTRAL_PWM;

    if (abs(raw_force) < critical_thrusts[type][MIN_THRUST])
        pwm = NEUTRAL_PWM;
    
    else if (raw_force > 0 && raw_force <= critical_thrusts[type][STARTUP_THRUST]) //  +Startup Thrust
        pwm = (int)(startup_config[type][POS_SLOPE] * raw_force + startup_config[type][POS_YINT]);

    else if (raw_force > 0 && raw_force > critical_thrusts[type][STARTUP_THRUST]) //  +Thrust
        pwm = (int)(primary_config[type][POS_SLOPE] * raw_force + primary_config[type][POS_YINT]);

    else if (raw_force < 0 && raw_force >= -critical_thrusts[type][STARTUP_THRUST]) //  -Startup Thrust
        pwm = (int)(startup_config[type][NEG_SLOPE] * raw_force + startup_config[type][NEG_YINT]);

    else if (raw_force < 0 && raw_force < -critical_thrusts[type][STARTUP_THRUST]) //  -Thrust
        pwm = (int)(primary_config[type][NEG_SLOPE] * raw_force + primary_config[type][NEG_YINT]);

    else
    {
        pwm = NEUTRAL_PWM;
    }

    //Constrain pwm output due to physical limitations of the ESCs
    if(pwm > MAX_PWM){
        pwm = MAX_PWM;
        RCLCPP_WARN(this->get_logger(), "Thruster Saturating. %s capped at MAX PWM = %i", name.c_str(), MAX_PWM);
    }
    if(pwm < MIN_PWM){
        pwm = MIN_PWM;
        RCLCPP_WARN(this->get_logger(), "Thruster Saturating. %s capped at MIN PWM = %i", name.c_str(), MIN_PWM);
    }

    return pwm;    
}

void PWMController::SwitchCB(cabin_msgs::msg::SwitchState state){
    dead = !state.kill;
}

void PWMController::ResetController(cabin_msgs::msg::ResetControls reset_msg){
    //if(reset_msg.reset_pwm)
    //   reset_pwm = ture;
    //else
    //   reset_pwm = false;
    if(reset_msg.reset_pwm)
        reset_pwm = true;
    else
        reset_pwm = true;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor pwm_controller_executor;
    auto pwm_controller_node = std::make_shared<PWMController>();
    pwm_controller_executor.add_node(pwm_controller_node);
    pwm_controller_executor.spin();
    pwm_controller_executor.remove_node(pwm_controller_node);
    return 0;
}