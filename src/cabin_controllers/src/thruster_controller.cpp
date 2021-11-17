#include "cabin_controllers/thruster_controller.hpp"

ThrusterController::ThrusterController()
: rclcpp::Node("thruster_controller", "cabin_auv"), count_(0){
    using std::placeholders::_1;
    ns = this->get_namespace();
    
    //Load parameters from .yaml files or launch files
    ThrusterController::LoadParam<string>("properties_file", propertiesFile);
    if(access(propertiesFile.c_str(), F_OK)){
        RCLCPP_ERROR(this->get_logger(), "%s Properties file \"%s\" does not exist", ns.c_str(), propertiesFile.c_str());
    }
    properties = YAML::LoadFile(propertiesFile);

    // Load buoyancy parameters from .yaml files or launch files
    ThrusterController::LoadParam<string>("buoyancy_properties_file", buoyancyPropertiesFile);
    if(access(buoyancyPropertiesFile.c_str(), F_OK)){
        RCLCPP_ERROR(this->get_logger(), "%s Properties file \"%s\" does not exist", ns.c_str(), buoyancyPropertiesFile.c_str());
    }
    buoyancyProperties = YAML::LoadFile(buoyancyPropertiesFile);

    // Load parameters from the files
    ThrusterController::LoadVehicleProperties();
    ThrusterController::SetThrusterCoeffs();
    
    // initial public parameters
    weightLoad_eig.setZero();
    for(int i = 0; i < 6; i++){
        weightLoad[i] = 0;
        transportThm[i] = 0;
        command[i] = 0;
        //solver_forces[i] = 0;
    } 
    for(int i = 0; i < 8; i++){
        solver_forces[i] = 0;
    }
    for(int i = 0; i < 3; i++){
        solver_cob[i] = 0;
        Fb_vector[i] = 0;
    }
    
    //dynamic parameters configuration  
    ThrusterController::DynamicParamConfigure();
    
    //subscriber and publisher
    state_sub = this->create_subscription<cabin_msgs::msg::Imu>(
         "state/imu", rclcpp::SensorDataQoS(), std::bind(&ThrusterController::ImuCB, this, _1));
    cmd_sub = this->create_subscription<cabin_msgs::msg::NetLoad>(
        "command/netLoad", rclcpp::SensorDataQoS(), std::bind(&ThrusterController::NetLoadCB, this, _1));
    thrust_pub = this->create_publisher<cabin_msgs::msg::ThrustStamped>("command/thrust", 1);
    timer_thrust_pub = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&ThrusterController::timer_callback_thrust_pub, this));
    //cob_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("properties/cob", 1);

    ThrusterController::InitThrustMsg();
    
    //EOM problem
    problemEOM.AddResidualBlock(new ceres::AutoDiffCostFunction<EOM, 6, 8>(new EOM(numThrusters, thrustCoeffs, inertia, weightLoad, transportThm, command)), NULL, solver_forces);
    optionsEOM.max_num_iterations = 100;
    optionsEOM.linear_solver_type = ceres::DENSE_QR;
    //Buoyancy Problem
    problemBuoyancy.AddResidualBlock(new ceres::AutoDiffCostFunction<FindCoB, 3, 3>(new FindCoB(numThrusters, thrustCoeffs, Fb_vector, solver_forces)), NULL, solver_cob);
    optionsBuoyancy.max_num_iterations = 100;
    optionsBuoyancy.linear_solver_type = ceres::DENSE_QR;
}

//Load paramter from namespace
template<typename T>
void ThrusterController::LoadParam(std::string param, T &var){
    this->declare_parameter<std::string>(param, "");
    bool param_get = this->get_parameter(param, var);
    if(!param_get){
        RCLCPP_ERROR(this->get_logger(), "Thruster Controller Namespace: %s", ns.c_str());
        RCLCPP_ERROR(this->get_logger(), "Critical! Param \"%s/%s\" does not exist or is not accessed correctly . Shutting down.", ns.c_str(), param.c_str());
        rclcpp::shutdown();
    }
}

void ThrusterController::LoadVehicleProperties(){
    mass = properties["properties"]["mass"].as<double>();

    double comX = properties["properties"]["center_of_mass"][0].as<double>();
    double comY = properties["properties"]["center_of_mass"][1].as<double>();
    double comZ = properties["properties"]["center_of_mass"][2].as<double>();
    center_of_mass[0] = comX;
    center_of_mass[1] = comY;
    center_of_mass[2] = comZ;

    Fg = mass * GRAVITY;
    depth_fully_submerged = properties["properties"]["depth_fully_submerged"].as<double>();

    Ixx = properties["properties"]["inertia"][0].as<double>();
    Iyy = properties["properties"]["inertia"][1].as<double>();
    Izz = properties["properties"]["inertia"][2].as<double>();

    inertia[0] = mass;
    inertia[1] = mass;
    inertia[2] = mass;
    inertia[3] = Ixx;
    inertia[4] = Iyy;
    inertia[5] = Izz;

    //init buoyancy parameters
    initBuoyantForce = buoyancyProperties["Buoyant_Force"].as<double>();
    initBuoyancyCenterX = buoyancyProperties["Buoyancy_X_POS"].as<double>();
    initBuoyancyCenterY = buoyancyProperties["Buoyancy_Y_POS"].as<double>();
    initBuoyancyCenterZ = buoyancyProperties["Buoyancy_Z_POS"].as<double>();
    initSubmerged = buoyancyProperties["IS_Submerged"].as<bool>();
}

void ThrusterController::SetThrusterCoeffs(){
    numThrusters = properties["properties"]["thrusters"].size();
    for(int i =0; i < numThrusters; i++){
        bool enabled = properties["properties"]["thrusters"][i]["enable"].as<bool>();
        thrustersEnabled.push_back((int)enabled);
    }

    //Each COLUMN contains a thruster's info
    int numThrustParams = properties["properties"]["thrusters"][0]["pose"].size();
    thrusters.resize(numThrustParams, numThrusters);
    thrustCoeffs.resize(6, numThrusters);
    thrusters.setZero();
    thrustCoeffs.setZero();

    for(int i = 0; i < numThrusters; i++){
        if(thrustersEnabled[i]){
            for(int j = 0; j < 5; j++){
                //Transform X, Y, Z to COM reference frame
                if(j < 3){
                    thrusters(j, i) = properties["properties"]["thrusters"][i]["pose"][j].as<double>() - center_of_mass[j];
                }
                else{
                    thrusters(j, i) = properties["properties"]["thrusters"][i]["pose"][j].as<double>();
                }
            }
        }
    }

    for(int i = 0; i < numThrusters; i++){
        if(thrustersEnabled[i]){
            //rotate around z, y axis
            float psi = thrusters(3, i) * PI / 180;
            float theta = thrusters(4, i) * PI / 180;
            thrustCoeffs(0, i) = cos(psi) * cos(theta);  //Effective contribution along X-axis
            thrustCoeffs(1, i) = sin(psi) * cos(theta);  //Effective contribution along Y-axis
            thrustCoeffs(2, i) = -sin(theta);            //Effective contribution along Z-axis

            //cross-product
            //Determine the effective moment arms for each thruster about the B-frame axes
            thrustCoeffs.block<3, 1>(3, i) = thrusters.block<3, 1>(0, i).cross(thrustCoeffs.block<3, 1>(0, i));
        }
    }
}

//Have not been tested since I have no imu then, please annotation this function if bug occur...
void ThrusterController::ImuCB(const cabin_msgs::msg::Imu imuMsg){
    float phi = imuMsg.rpy_deg.x * PI /180;
    float theta = imuMsg.rpy_deg.y * PI / 180;

    Vector3d angular_vel;
    angular_vel[0] = imuMsg.ang_vel_deg.x * PI / 180;
    angular_vel[1] = imuMsg.ang_vel_deg.y * PI / 180;
    angular_vel[2] = imuMsg.ang_vel_deg.z * PI / 180;

    transportThm[3] = -angular_vel[1] * angular_vel[2] * (Izz - Iyy);
    transportThm[4] = -angular_vel[0] * angular_vel[2] * (Ixx - Izz);
    transportThm[5] = -angular_vel[0] * angular_vel[1] * (Iyy - Ixx);
    
    Vector3d Fb_eig;
    Fb_eig(0) = Fb * sin(theta);
    Fb_eig(1) = -Fb * sin(phi) * cos(theta);
    Fb_eig(2) = -Fb * cos(phi) * cos(theta);

    weightLoad_eig(0) = -(Fg - Fb) * sin(theta);
    weightLoad_eig(1) = (Fg - Fb) * sin(phi) * cos(theta);
    weightLoad_eig(2) = (Fg - Fb) * cos(phi) * cos(theta);
    weightLoad_eig.segment<3>(3) = CoB.cross(Fb_eig);
    weightLoad_eig = weightLoad_eig * ((int)(isSubmerged));

    //Convert Eigen::VectorXd to c++ double[X]
    Map<RowMatrixXd>(&weightLoad[0], weightLoad_eig.rows(), weightLoad_eig.cols()) = weightLoad_eig;
    Map<Vector3d>(&Fb_vector[0], Fb_eig.rows(), Fb_eig.cols()) = Fb_eig;
}

void ThrusterController::NetLoadCB(const cabin_msgs::msg::NetLoad load_msg){
    command[0] = load_msg.force.x;
    command[1] = load_msg.force.y;
    command[2] = load_msg.force.z;
    command[3] = load_msg.moment.x;
    command[4] = load_msg.moment.y;
    command[5] = load_msg.moment.z;

    //These initial guesses don't make much of a difference
    for(int i = 0; i < numThrusters; i++){
        solver_forces[i] = 0.0;
    }

    //solve all problems
    ceres::Solve(optionsEOM, &problemEOM, &summaryEOM);

    rclcpp::Time now = this->now();
    thrust_msg.header.stamp = now;
    thrust_msg.force.vector_port_fwd = solver_forces[thrust_msg.force.VPF];
    thrust_msg.force.vector_stbd_fwd = solver_forces[thrust_msg.force.VSF];
    thrust_msg.force.vector_port_aft = solver_forces[thrust_msg.force.VPA];
    thrust_msg.force.vector_stbd_aft = solver_forces[thrust_msg.force.VSA];
    thrust_msg.force.heave_port_fwd = solver_forces[thrust_msg.force.HPF];
    thrust_msg.force.heave_stbd_fwd = solver_forces[thrust_msg.force.HSF];
    thrust_msg.force.heave_port_aft = solver_forces[thrust_msg.force.HPA];
    thrust_msg.force.heave_stbd_aft = solver_forces[thrust_msg.force.HSA];

    //thrust_pub->publish(thrust_msg);
    
}

void ThrusterController::timer_callback_thrust_pub(){
    rclcpp::Time now = this->now();
    thrust_msg.header.stamp = now;
    thrust_pub->publish(thrust_msg);
}

void ThrusterController::InitThrustMsg(){
    rclcpp::Time now = this->now();
    thrust_msg.header.stamp = now;
    thrust_msg.force.vector_port_fwd = 0;
    thrust_msg.force.vector_stbd_fwd = 0;
    thrust_msg.force.vector_port_aft = 0;
    thrust_msg.force.vector_stbd_aft = 0;
    thrust_msg.force.heave_port_fwd = 0;
    thrust_msg.force.heave_stbd_fwd = 0;
    thrust_msg.force.heave_port_aft = 0;
    thrust_msg.force.heave_stbd_aft = 0;

    thrust_pub->publish(thrust_msg);
}

void ThrusterController::DynamicParamConfigure(){
    //dynamic parameters configuration   
    dynamicReconfigure = std::make_shared<rclcpp::ParameterEventHandler>(this);
    //set the buoyant force
    rcl_interfaces::msg::FloatingPointRange buoyantForceRange;
    buoyantForceRange.from_value = 0.0;
    buoyantForceRange.step = 1.0;
    buoyantForceRange.to_value = 500.0;
    rcl_interfaces::msg::ParameterDescriptor buoyantForceDescriptor;
    buoyantForceDescriptor.description = "set the buoyant force";
    buoyantForceDescriptor.floating_point_range.push_back(buoyantForceRange);
    this->declare_parameter("Buoyant_Force", rclcpp::ParameterValue(initBuoyantForce), buoyantForceDescriptor);
    auto buoyantForceCB = [this](const rclcpp::Parameter & parameter){
        Fb = parameter.as_double();
    };
    buoyantForceHandle = dynamicReconfigure->add_parameter_callback("Buoyant_Force", buoyantForceCB);

    //set the buoyancy center
    this->declare_parameter("Buoyancy_X_POS", initBuoyancyCenterX);
    this->declare_parameter("Buoyancy_Y_POS", initBuoyancyCenterY);
    this->declare_parameter("Buoyancy_Z_POS", initBuoyancyCenterZ);
    auto buoyancyCenterXCB = [this](const rclcpp::Parameter & parameter){
        CoB(0) = parameter.as_double();
    };  
    auto buoyancyCenterYCB = [this](const rclcpp::Parameter & parameter){
        CoB(1) = parameter.as_double();
    };
    auto buoyancyCenterZCB = [this](const rclcpp::Parameter & parameter){
        CoB(2) = parameter.as_double();
    };
    buoyancyCenterXHandle = dynamicReconfigure->add_parameter_callback("Buoyancy_X_POS", buoyancyCenterXCB); 
    buoyancyCenterYHandle = dynamicReconfigure->add_parameter_callback("Buoyancy_Y_POS", buoyancyCenterYCB);
    buoyancyCenterZHandle = dynamicReconfigure->add_parameter_callback("Buoyancy_Z_POS", buoyancyCenterZCB);

    //set the submerged state
    this->declare_parameter<bool>("Is_Submerged", initSubmerged);
    auto isSubmergedCB = [this](const rclcpp::Parameter & parameter){
        isSubmerged = parameter.as_bool();
    };
    submergedHandle = dynamicReconfigure->add_parameter_callback("Is_Submerged", isSubmergedCB);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor thrust_controller_executor;
    auto thrust_controller_node = std::make_shared<ThrusterController>();
    thrust_controller_executor.add_node(thrust_controller_node);
    thrust_controller_executor.spin();
    thrust_controller_executor.remove_node(thrust_controller_node);
    return 0;
}