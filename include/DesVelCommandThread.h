#ifndef DesVelCommandThread_H
#define DesVelCommandThread_H


#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl2.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;

using namespace std;
using namespace Eigen;



class DesVelCommandThread : public RateThread
{
     


public:

    std::string moduleName;
    std::string robotName;
    int ThreadPeriod;

    // internal number of cycles indicator
    int CycleCounter;
    int SwitchCounter;

    // starting time
    double start_time;

    // Creating port for keyboard input cmd
    Bottle *keyboardValues;
    BufferedPort<Bottle> VelocityCmd_port_In;
    VectorXd des_COM_velo;  // factor of velocity increase

    DesVelCommandThread(int period, std::string _moduleName, std::string _robotName);
    // ~DesVelCommandThread();

    bool threadInit();

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void threadRelease();
    
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Run method
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void run();
    

};

#endif //DesVelCommandThread_H