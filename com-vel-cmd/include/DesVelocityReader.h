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

        string            moduleName_;
        Eigen::VectorXd   curr_COM_;
        Eigen::Vector3d   init_vel_;
        int               VelocityCmdType_;

        // Creating port for keyboard input cmd
        Bottle *keyboardValues;
        BufferedPort<Bottle> KeyboardCmd_port_In;

        // For Keyboard Input
        Eigen::VectorXd   incr_vel_;    // factor of velocity increase

        // For DS input from ROS-YARP translator
        //....
  

    public:
    
        
        Eigen::VectorXd des_com_vel_;  // factor of velocity increase        
        
        DesVelocityReader(string moduleName, int VelocityCmdType, Eigen::Vector3d  init_vel);

        bool initReader();

        void stop();

        void read();    

};
