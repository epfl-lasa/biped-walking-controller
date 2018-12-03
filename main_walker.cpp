#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
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

#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <ncurses.h>
#include <math.h>

#include "InitBalWlkParameters.h"
#include "CpMath_Utilities.h"
#include "CommunicationControl.h"
#include "RobotModel.h"
#include "TemplateModels.h"
#include "MPCOptimizer.h"
#include "PatternsGenerator.h"
#include "EstimatorCompensators.h"
#include "Data_logging.h"

#include "ReferencesCompensator.h"
#include "Grasping.h"
#include "OptimalFilters.h"

#include "CpBalWlkCtrlThread.h"

#include <qpOASES.hpp>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;

using namespace std;
using namespace Eigen;



int main(int argc, char **argv) //(int argc, char *argv[])
{
    
    // Establish connection with yarp network
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        printf("No yarp network, quitting\n");
        return 1;
    }
    // ====================================================================================
    // creation of a ressource finder object
    yarp::os::ResourceFinder rf; // = yarp::os::ResourceFinder::getResourceFinderSingleton();    
    rf.setVerbose(true);                                        //logs searched directories
    rf.setDefaultConfigFile("BalanceWalkingController.ini");    //default config file name.
    rf.configure(argc, argv);
    
    if (rf.check("help")) 
    {
        std::cout<< "Possible parameters" << std::endl << std::endl;
        std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /BalanceWalkingController/config" << std::endl;
        std::cout<< "\t--rate             :Period used by the module. Default set to 10ms." << std::endl;
        std::cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default." << std::endl;
        std::cout<< "\t--moduleName       :name of the module ." << std::endl;
        std::cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyHPIDControl." << std::endl;
        return 0;
    }

    //Loading configuration file information
    yarp::os::Property wbProperties;

    if (!rf.check("wbi_config_file", "Checking wbi configuration file")) 
    {
        yError("No WBI configuration file found.");
        std::printf("No WBI configuration file found. \n");
        return -1;
    }

    //Loading joints information
    if (!rf.check("wbi_joint_list", "Checking wbi joint list name \n")) 
    {
        //yError("No joint list found. Please specify a joint list in \"wbi_joint_list\"");
        std::printf("No joint list found. Please specify a joint list in \"wbi_joint_list\" \n");
        return -1;
    }

    // extract and print the configuration file
    yarp::os::ConstString initFile = rf.find("wbi_config_file").asString();
    std::cout<< "wbi_config_file: " << initFile.c_str() <<std::endl;

    // locate and load the configuration file indicated in the .ini config file
    if (!wbProperties.fromConfigFile(rf.findFile("wbi_config_file"))) 
    {
        //yError("Not possible to load WBI properties from file.");
        std::printf("Not possible to load WBI properties from file. \n");
        return -1;
    }

    //
    wbProperties.fromString(rf.toString(), false);

    // get the list of all the joints of the robot model
    yarp::os::ConstString jointList = rf.find("wbi_joint_list").asString();

    //retrieve all main joints
    wbi::IDList robotMainJoints;
    if (!yarpWbi::loadIdListFromConfig(jointList, wbProperties, robotMainJoints)) 
    {
        yError("Cannot find joint list");
        return -1;
    }

    //Load configuration-time parameters
    std::string m_moduleName = rf.check("name", Value("CpBalanceWalkingController"), "Looking for module name").asString();
    std::string m_robotName = rf.check("robot", Value("icubGazeboSim"), "Looking for robot name").asString();
    int period          = rf.check("period", Value(40), "Looking for controller period").asInt();
    double RunDuration   = rf.check("duration", Value(5), "Looking for Running Duration ").asDouble();
    int FT_feedbackType  = rf.check("FT_feedback", Value(0), "Looking for Running Duration ").asInt();
    int KeyBoardCommand = rf.check("KeyBoardCommand", Value(0), "Looking for keyboard input ").asInt();

    Eigen::Vector3d InitVelocity;
    InitVelocity(0) = rf.check("VelocityX", Value(0.10), "Looking for forward velocity ").asDouble();
    InitVelocity(1) = rf.check("VelocityY", Value(0), "Looking for lateral velocity ").asDouble();
    InitVelocity(2) = rf.check("OmegaZ", Value(0), "Looking for rotation velocity ").asDouble();

    // convert the period from millisecond to second
    double m_period = 0.001 * (double) period;

    std::cout << " the duration is : \n"  << RunDuration << std::endl;

    // creation of the wbi robot object
    // =================================
    wbi::wholeBodyInterface* robot;
    robot = new yarpWbi::yarpWholeBodyInterface(m_moduleName.c_str(), wbProperties);

    // add joints
    robot->addJoints(robotMainJoints);
    // calling the init method
    if (!robot->init()) 
    {
        yError("Could not initialize wbi.");
        return -1;
    }

    Time::delay(0.5);   

    // control of the robot through keyboard
    bool ActiveKeyBoardCtrl = false;
    if(KeyBoardCommand == 1)
    {
        ActiveKeyBoardCtrl = true;
    }
    else{
        ActiveKeyBoardCtrl = false;
    }  

    // Instantiate the Control Thread
    CpBalWlkCtrlThread myCtrlThread(period, m_moduleName, m_robotName, FT_feedbackType, ActiveKeyBoardCtrl, *robot); //period is 40ms

    // Starting the BalanceWalkingController Thread
    myCtrlThread.start();

     // variables for end of walking configuration
    int n_Samp1, n_Samp_init;
        n_Samp1 = (int)(round(myCtrlThread.Parameters->DurationSteps[0]/myCtrlThread.Parameters->SamplingTime));


    // Set the Initial Desired CoM velocity
    myCtrlThread.Des_RelativeVelocity = InitVelocity;
    bool done=false;
    bool finalConf = false;
    double startTime=Time::now();
    double FinalMoveTime = Time::now();

    // variable for feet alignment and stopping
    bool AlignFeet = false;
    bool noInput   = false;


    // =================================
    // Creating port for keyboard input cmd
    Bottle *keyboardValues;
    BufferedPort<Bottle> KeyboardCmd_port_In;
    VectorXd incr_vel;   // factor of velocity increase
    VectorXd des_com_vel;  // factor of velocity increase

    //Opening the port for the Keyboard input cmds  moduleName
    std::string KeyboardOutputs_port ="/";
                KeyboardOutputs_port += m_moduleName;
                KeyboardOutputs_port += "/keyboardOutputs:o";


    KeyboardCmd_port_In.open("/KeyboardInputCmds:i");

    if(!Network::connect(KeyboardOutputs_port.c_str(), KeyboardCmd_port_In.getName().c_str()))
    {
        printf(" Unable to connect to the KeyboardCmdsReaderModule port");
        return false;
    }

    // Reading of measurement
    keyboardValues  = KeyboardCmd_port_In.read(); 
    incr_vel.resize(keyboardValues->size());
    incr_vel.setZero();

    des_com_vel.resize(keyboardValues->size());
    des_com_vel.setZero();

    des_com_vel(0) = InitVelocity(0);
    des_com_vel(1) = InitVelocity(1);
    des_com_vel(2) = InitVelocity(2);

    while(!(done && finalConf) && !myCtrlThread.StopCtrl && !isnan(incr_vel(0)))
    {
        // Extract the read value
        keyboardValues  = KeyboardCmd_port_In.read();         
        incr_vel(0) = keyboardValues->get(0).asDouble();
        incr_vel(1) = keyboardValues->get(1).asDouble();
        incr_vel(2) = keyboardValues->get(2).asDouble();
        printf("Desired Keyboard Command +vx:%4.6f +vy:%4.6f  +wz:%4.6f \n", incr_vel(0), incr_vel(1), incr_vel(2));

        if(myCtrlThread.KeyboardCtrl) //((c= getch())!=27)
        {   

            // Add increments
            des_com_vel(0) = des_com_vel(0) + incr_vel(0);
            des_com_vel(1) = des_com_vel(1) + incr_vel(1);
            des_com_vel(2) = des_com_vel(2) + incr_vel(2);


            // Truncate velocities
            // forward and backward walking 
            // -----------------------------         
            if(des_com_vel(0) >= 0.1 && incr_vel(0) > 0.0) { des_com_vel(0) = 0.1; }
            if(des_com_vel(0) <= -0.1 && incr_vel(0) < 0.0) {  des_com_vel(0) = -0.1;  }


            // lateral walking 
            // ---------------
            if(des_com_vel(1) >= 0.1 && incr_vel(1) > 0.0) { des_com_vel(1) = 0.1; }
            if(des_com_vel(1) <= -0.1 && incr_vel(1) < 0.0) {  des_com_vel(1) = -0.1;  }

            // rotation
            // --------
            if(des_com_vel(2) >= 0.2 && incr_vel(2) > 0.0) { des_com_vel(2) = 0.2; }
            if(des_com_vel(2) <= -0.2 && incr_vel(2) < 0.0) {  des_com_vel(2) = -0.2; }
        

            printf("Sent Desired Velocity vx:%4.6f vy:%4.6f  wz:%4.6f \n", des_com_vel(0), des_com_vel(1), des_com_vel(2));
            

            myCtrlThread.Des_RelativeVelocity(0) = des_com_vel(0);
            myCtrlThread.Des_RelativeVelocity(1) = des_com_vel(1);
            myCtrlThread.Des_RelativeVelocity(2) = des_com_vel(2);            

        }
        else
        {
            // Set the Desired CoM velocity
            myCtrlThread.Des_RelativeVelocity(0) = 0.0;
            myCtrlThread.Des_RelativeVelocity(1) = 0.0;
            myCtrlThread.Des_RelativeVelocity(2) = 0.0; 
        }

      
        if (!done &&((Time::now()-startTime)> RunDuration))
        {

            done=true;

            FinalMoveTime = Time::now();

            n_Samp_init = 2*(n_Samp1 - 1) - myCtrlThread.CpBalWlkController->SMx->IndexSFt; // 2*

            myCtrlThread.Des_RelativeVelocity(0) = 0.00;
            myCtrlThread.Des_RelativeVelocity(1) = 0.00;
            myCtrlThread.Des_RelativeVelocity(2) = 0.00;

            cout << " SamplingTime : "<< myCtrlThread.Parameters->SamplingTime << endl;
            cout << " Alignment bool : "<< (done) << endl;
            //finalConf = true;
        }

        if (done)
        {
            myCtrlThread.Des_RelativeVelocity(0) = 0.00;
            myCtrlThread.Des_RelativeVelocity(1) = 0.00;
            myCtrlThread.Des_RelativeVelocity(2) = 0.00;
        }

        if (done && ((Time::now()-FinalMoveTime)>= (n_Samp_init *myCtrlThread.Parameters->SamplingTime)))
        {
            finalConf = true;

            cout << " Feet Alignment done, Now closing " << endl;
        }    

    }

    // Set to Zero the Desired CoM velocity before stoping
    myCtrlThread.Des_RelativeVelocity(0) = 0.00;  // TO DO
    myCtrlThread.Des_RelativeVelocity(1) = 0.00;
    myCtrlThread.Des_RelativeVelocity(2) = 0.00;
    
    myCtrlThread.stop();

    // Close keyboard thread
    KeyboardCmd_port_In.close();

    // close the wholebodyInterface object (robot)
    if (robot) 
    {
        robot->close();
        delete robot;
        robot = 0;
    }


    return 0;
}
