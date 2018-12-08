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


using namespace std;
using namespace yarp::os;

class DesVelocityCommandROS
{
     

    private: 

        string               moduleName_;
        string               robotName_;        
        int                  DSType_;
        double               max_v, max_w, kappa_;

        // Creating port for keyboard input cmd
        Bottle              *keyboardValues;
        BufferedPort<Bottle> KeyboardCmd_port_In;


        // Creating port for reading Root-link in world frame pose
        Bottle              *RootlinkPose_values;
        BufferedPort<Bottle> RootlinkPose_port_In;
        Eigen::VectorXd      Rootlink_measurements;
        Eigen::Vector3d      CoM_pos;
        Eigen::Vector3d      CoM_orient_rpy;
        Eigen::Matrix3d      CoM_orient_rot;
        Eigen::Quaterniond   CoM_orient_quat;

        // Creating port for publishing CoM pose of robot in world frame
        BufferedPort<yarp::sig::Vector> CoMPose_port_Out;
        yarp::sig::Vector               CoMPoses;


        // Creating port for reading DS desired velocity from ROS
        Bottle              *DSVelocity_values;
        BufferedPort<Bottle> DSVelocity_port_In;


        // Creating port for reading attractor from ROS
        Bottle              *DSAttractor_values;
        BufferedPort<Bottle> DSAttractor_port_In;

        // For linear DS input
        Eigen::Vector3d      DS_desired_velocity_;
        Eigen::Vector3d      attractor_;   


    public:
    
        
        Eigen::VectorXd des_com_vel_;
        
        DesVelocityCommandROS(string moduleName, string robotName, int DSType);

        bool initReader();

        void stop();

        void updateCoM();    

        void updateDesComVel();    

        void setAttractor(Eigen::Vector3d attractor);

        void setkappa(double kappa);

        Eigen::Vector3d linearDS();

        Eigen::Vector3d DSfromROS();

        double computeAngularVelocity(Eigen::Vector3d x_dot);

};
