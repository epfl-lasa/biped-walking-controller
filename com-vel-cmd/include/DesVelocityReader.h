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

using namespace std;
using namespace yarp::os;

class DesVelocityReader
{
     

    private: 

        string               moduleName_;
        string               robotName_;        
        int                  VelocityCmdType_;

        // Creating port for keyboard input cmd
        Bottle              *keyboardValues;
        BufferedPort<Bottle> KeyboardCmd_port_In;
        Eigen::Vector3d      init_vel_;    // initial velocity set by user
        Eigen::VectorXd      incr_vel_;    // factor of velocity increase


        // Creating port for Root-link in world frame pose
        Bottle              *RootlinkPose_values;
        BufferedPort<Bottle> RootlinkPose_port_In;
        Eigen::VectorXd      Rootlink_measurements;

        // For DS input from ROS-YARP translator
        //....
  

    public:
    
        
        Eigen::VectorXd des_com_vel_;  // factor of velocity increase        
        
        DesVelocityReader(string moduleName, string robotName, int VelocityCmdType, Eigen::Vector3d  init_vel);

        bool initReader();

        void stop();

        void updateCoM();    

        void updateDesComVel();    

};
