#include <iostream>
#include <iomanip>
#include <math.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/BufferedPort.h>
#include <ncurses.h>


#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

/* Include Library Headers */
#include "lpvDS.h"
#include "utils.h"

using namespace std;
using namespace yarp::os;

class DesVelocityCommand_lpvDS
{
     

    private: 

        string               moduleName_;
        string               robotName_;   
        int                  DSType_, counter_;
        double               max_v, max_w, kappa_;
        double               rot_angle_, dist_thres_, des_omega_;

        // Creating port for keyboard input cmd
        Bottle              *keyboardValues;
        BufferedPort<Bottle> KeyboardCmd_port_In;


        // Creating port for reading Root-link in world frame pose
        Bottle              *RootlinkPose_values;
        BufferedPort<Bottle> RootlinkPose_port_In;
        Eigen::VectorXd      Rootlink_measurements;        
        Eigen::Matrix3d      CoM_orient_rot;
        Eigen::Quaterniond   CoM_orient_quat;

        // Creating port for publishing CoM pose of robot in world frame
        BufferedPort<yarp::sig::Vector> CoMPose_port_Out;
        yarp::sig::Vector               CoMPoses;


        // Creating port for reading DS desired velocity from ROS
        // Bottle              *DSVelocity_values;
        // BufferedPort<Bottle> DSVelocity_port_In;


        // Creating port for reading attractor from ROS
        // Bottle              *DSAttractor_values;
        // BufferedPort<Bottle> DSAttractor_port_In;

        // For linear DS input
        Eigen::Vector3d      DS_desired_velocity_;
        Eigen::Vector3d      attractor_;   


        // Variables for lpvDS class
        string                  path_model_; 
        string                  path_to_models_ ;   
        string                  lpvDS_model_name_;    
        std::unique_ptr<lpvDS>  LPV_DS_;
        VectorXd                att_;
        fileUtils               fileUtils_;

    public:

        Eigen::Vector3d      CoM_pos;
        Eigen::Vector3d      CoM_orient_rpy;
        Eigen::VectorXd      des_com_vel_;
        
        DesVelocityCommand_lpvDS(string moduleName, string robotName, int DSType, string path_to_models, string lpvDS_model_name);

        bool initReader();

        void stop();

        void updateCoM();    

        void updateDesComVel();    

        void setAttractor(Eigen::Vector3d attractor);

        void setkappa(double kappa);

        Eigen::Vector3d linearDS();

        // Eigen::Vector3d DSfromROS();

        Eigen::Vector3d DSfromLPVDS();
        
        Eigen::Vector3d RotateDS(Eigen::Vector3d x_dot);

        double computeAngularVelocity(Eigen::Vector3d x_dot);

};
