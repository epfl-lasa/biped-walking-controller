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

class DesVelocityReader
{
     

    private: 

        string               moduleName_;
        string               robotName_;        
        int                  VelocityCmdType_;
        double               max_v, max_w;

        // Creating port for keyboard input cmd
        Bottle              *keyboardValues;
        BufferedPort<Bottle> KeyboardCmd_port_In;
        Eigen::Vector3d      init_vel_;    // initial velocity set by user
        Eigen::VectorXd      incr_vel_;    // factor of velocity increase


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

        // For DS input
        Eigen::Vector3d      attractor_;    // initial velocity set by user


    public:
    
        
        Eigen::VectorXd des_com_vel_;
        
        DesVelocityReader(string moduleName, string robotName, int VelocityCmdType, Eigen::Vector3d  init_vel);

        bool initReader();

        void stop();

        void updateCoM();    

        void updateDesComVel();    

        void setAttractor(Eigen::Vector3d attractor);

        Eigen::Vector3d linearDS(double kappa);

        double computeAngularVelocity(Eigen::Vector3d x_dot);

};
