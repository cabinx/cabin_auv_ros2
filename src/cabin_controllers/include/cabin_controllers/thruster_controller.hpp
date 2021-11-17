#ifndef THRUSTER_CONTROLLER_HPP
#define THRUSTER_CONTROLLER_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rcl_interfaces/msg/parameter_event.hpp>

//#include <fstream>
#include <unistd.h>
#include "yaml-cpp/yaml.h"

#include "math.h"
#include "ceres/ceres.h"

#include "cabin_msgs/msg/thrust_stamped.hpp"
#include "cabin_msgs/msg/imu.hpp"
#include "cabin_msgs/msg/net_load.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;
using namespace std;

#define GRAVITY 9.81    //[m/s^2]
#define PI 3.141592653
#define WATER_DENSITY 1000  //[kg/m^3]

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, Dynamic, RowMajor> RowMatrixXd;

class ThrusterController : public rclcpp::Node{
    public:
        ThrusterController();
        virtual ~ThrusterController() = default;
    
    private:
        std::string ns;               //namespace 
        YAML::Node properties;
        YAML::Node buoyancyProperties;
        string propertiesFile;
        string buoyancyPropertiesFile;
        vector<int> thrustersEnabled;
        Vector3d CoB;
        double mass, Fg, Fb, Ixx, Iyy, Izz, depth_fully_submerged;
        bool isSubmerged;

        //Variables that get passed to class EOM and FindCoB
        MatrixXd thrustCoeffs, thrusters;
        Vector6d weightLoad_eig;
        int numThrusters;
        double inertia[6], weightLoad[6], transportThm[6], command[6], Fb_vector[3];
        double solver_forces[8];   //Solved forces go here
        double solver_cob[3];      //Solved buoyancy positions go here
        double center_of_mass[3];

        //EOMS
        ceres::Problem problemEOM;
        ceres::Solver::Options optionsEOM;
        ceres::Solver::Summary summaryEOM;

        //Locate Buoyancy Position
        ceres::Problem problemBuoyancy;
        ceres::Solver::Options optionsBuoyancy;
        ceres::Solver::Summary summaryBuoyancy;

        rclcpp::Subscription<cabin_msgs::msg::Imu>::SharedPtr state_sub;
        rclcpp::Subscription<cabin_msgs::msg::NetLoad>::SharedPtr cmd_sub;
        rclcpp::Publisher<cabin_msgs::msg::ThrustStamped>::SharedPtr thrust_pub;
        rclcpp::TimerBase::SharedPtr timer_thrust_pub;
        void timer_callback_thrust_pub();
        //rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr cob_pub;
        
        template<typename T>
        void LoadParam(std::string param, T &var);
        void LoadVehicleProperties();
        void SetThrusterCoeffs();

        void ImuCB(const cabin_msgs::msg::Imu imu_msg);
        void NetLoadCB(const cabin_msgs::msg::NetLoad load_msg);

        void InitThrustMsg();
        void DynamicParamConfigure();

        cabin_msgs::msg::ThrustStamped thrust_msg;
        geometry_msgs::msg::Vector3Stamped cob_msg;

        //parameters dynamic reconfiguration
        std::shared_ptr<rclcpp::ParameterEventHandler> dynamicReconfigure;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> buoyantForceHandle;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> buoyancyCenterXHandle;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> buoyancyCenterYHandle;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> buoyancyCenterZHandle;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> submergedHandle;
        double initBuoyantForce;
        double initBuoyancyCenterX;
        double initBuoyancyCenterY;
        double initBuoyancyCenterZ;
        double initSubmerged;
        size_t count_;
};

//Class EOM defines the 6 equations of motiion that ceres needs to solve
class EOM{
    private:
        int numThrusters;           //Number of thrusters
        MatrixXd thrustCoeffs;      //Thrust coefficients(effective contributions of each thruster for force and moments)
        double *inertia;            //(Pointer) Inertia-related values (mass, mass, Ixx, Iyy, Izz)
        double *weightLoad;         //(Pointer) Forces/moments due to weight forces (gravity and buoyancy)
        double *transportThm;       //(Pointer) Vector containng terms related to the transport theorem
        double *command;            //Command for each EOM

    public:
        EOM(int &numThrust, const Ref<const MatrixXd> &thrustCoeffsIn, double *inertiaIn, double *weightLoadIn, double *transportThmIn, double *commandIn){
            numThrusters = numThrust;
            thrustCoeffs = thrustCoeffsIn;
            inertia = inertiaIn;
            weightLoad = weightLoadIn;
            transportThm = transportThmIn;
            command = commandIn;
        }

        template <typename T>
        bool operator()(const T *const forces, T *residual) const{
            for (int i = 0; i < 6; i++){
                residual[i] = T(0);

                //Account for each thruster's contributioin to force/moment
                for(int j = 0; j < numThrusters; j++){
                    residual[i] = residual[i] + T(thrustCoeffs(i,j)) * forces[j];
                }

                //Account for weight-related forces/moments and transportThm
                residual[i] = residual[i] + T(weightLoad[i] + transportThm[i]);
                residual[i] = residual[i] - T(command[i]);
            }
            return true;
        }
};

//Class FindCoB will determine an estimate for the location of the center of buoyancy relative to the center of mass
class FindCoB{
    private:
        int numThrusters;        //Number of thrusters
        int x, y, z;             //Axis indeces
        MatrixXd thrustCoeffs;   //Thrust coefficients(effective contributions of each thruster for force and moments)
        double *Fb;              //(Pointer) Buoyant forces along body-frame x, y, and z axes
        double *forces;          //(Pointer) Solved thruster forces from class EOM

    public:
        FindCoB(int &numThrust, const Ref<const MatrixXd> &thrustCoeffsIn, double *FbIn, double *forcesIn){
            numThrusters = numThrust;
            thrustCoeffs = thrustCoeffsIn;
            Fb = FbIn;
            forces = forcesIn;
            x = 0, y = 1, z =2;
        }
        //rFb is the position vector for the center of buoyancy from the center of mass
        template<typename T>
        bool operator()(const T *const rFb, T *residual) const{
            for(int i = 0; i < 3; i++){
                residual[i] = T(0);
                for(int j = 0; j < numThrusters; j++){
                    residual[i] = residual[i] +T(thrustCoeffs(i+3, j) * forces[j]);
                }
            }
            residual[x] = residual[x] + (rFb[y] * T(Fb[z]) - rFb[z] * T(Fb[y]));
            residual[y] = residual[y] + (rFb[z] * T(Fb[x]) - rFb[x] * T(Fb[z]));
            residual[z] = residual[z] + (rFb[x] * T(Fb[y]) - rFb[y] * T(Fb[x]));
            return true;
        }
};
#endif