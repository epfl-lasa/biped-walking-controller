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
#include "DesVelocityCommandROS.h"

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
    // Creation of a resource finder object
    // ====================================================================================
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

    // Load configuration-time parameters
    std::string m_moduleName = rf.check("name",           Value(""), "Looking for module name").asString();
    std::string m_robotName  = rf.check("robot",          Value(""), "Looking for robot name").asString();
    int period               = rf.check("period",         Value(40), "Looking for controller period").asInt();
    double RunDuration       = rf.check("duration",       Value(5),  "Looking for Running Duration ").asDouble();
    int FT_feedbackType      = rf.check("FT_feedback",    Value(0),  "Looking for Running Duration ").asInt();
    int DSType               = rf.check("DSType",         Value(0),  "Looking for DStype ").asInt();


    // Load Parameters for DS_type=0 (Linear DS)
    Eigen::Vector3d Attractor;
    double kappa  = rf.check("kappa",     Value(0.005),   "kappa").asDouble();
    Attractor(0)  = rf.check("AttractorX", Value(0.0), "Attractor x position").asDouble();
    Attractor(1)  = rf.check("AttractorY", Value(0.0), "Attractor y position").asDouble();
    Attractor(2)  = rf.check("AttractorZ", Value(0.0), "Attractor z position").asDouble();    

    // convert the period from millisecond to second
    double m_period = 0.001 * (double) period;
    std::cout << " the duration is : \n"  << RunDuration << std::endl;

    // =================================
    // Creation of the wbi robot object
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

    // =================================
    // Instantiate the Control Thread
    // =================================
    CpBalWlkCtrlThread myCtrlThread(period, m_moduleName, m_robotName, FT_feedbackType, true, *robot); //period is 40ms

    // Starting the BalanceWalkingController Thread
    myCtrlThread.start();

     // variables for end of walking configuration
    int n_Samp1, n_Samp_init;
        n_Samp1 = (int)(round(myCtrlThread.Parameters->DurationSteps[0]/myCtrlThread.Parameters->SamplingTime));

    // Set the Initial Desired CoM velocity        
     Eigen::Vector3d InitVelocity; InitVelocity.setZero();
    myCtrlThread.Des_RelativeVelocity = InitVelocity;
    bool done=false;
    bool finalConf = false;
    double startTime=Time::now();
    double FinalMoveTime = Time::now();

    // variable for feet alignment and stopping
    bool AlignFeet = false;
    bool noInput   = false;

    // =======================================================
    // Instantiate the Desired CoM Velocity Command Generator 
    // =======================================================
    DesVelocityCommandROS myDesiredCoM( m_moduleName, m_robotName, DSType); 
    myDesiredCoM.initReader();
    myDesiredCoM.setAttractor(Attractor);        

    if (DSType == 0){
        myDesiredCoM.setAttractor(Attractor);        
        myDesiredCoM.setkappa(kappa);
    }

    // =======================================================
    //                  MAIN CONTROL LOOP
    // =======================================================
    // Variable for Desired CoM Velocity Given my port
    Eigen::VectorXd des_com_vel, CoM_pos, CoM_rpy;
    des_com_vel.resize(3); des_com_vel.setZero();
    CoM_pos.resize(3); CoM_pos.setZero();
    CoM_rpy.resize(3); CoM_rpy.setZero();

    // Open file
//    std::ofstream measurements_file;
//    measurements_file.open ("simulation_measurements.txt");

    while(!(done && finalConf) && !myCtrlThread.StopCtrl && !isnan(des_com_vel(0))){

        // Read the current desired CoM
        myDesiredCoM.updateDesComVel();


        // Get current CoM pose
        CoM_pos     = myDesiredCoM.CoM_pos;
        CoM_rpy     = myDesiredCoM.CoM_orient_rpy;

        // Get current desired velocity command
        des_com_vel = myDesiredCoM.des_com_vel_;        

        // Write data to file
//        measurements_file << CoM_pos(0) << " " << CoM_pos(1) << " " << CoM_pos(2) << " " << CoM_rpy(0) << " " << CoM_rpy(1) << " " << CoM_rpy(2) << " " << des_com_vel(0) << " " << des_com_vel(1) << " " << des_com_vel(2) <<"\n";

        // Send to controller thread 
        myCtrlThread.Des_RelativeVelocity(0) = des_com_vel(0);
        myCtrlThread.Des_RelativeVelocity(1) = des_com_vel(1);
        myCtrlThread.Des_RelativeVelocity(2) = des_com_vel(2);            

        // Stopping conditions
        if (!done &&((Time::now()-startTime)> RunDuration)){

            done=true;
            FinalMoveTime = Time::now();
            n_Samp_init = 2*(n_Samp1 - 1) - myCtrlThread.CpBalWlkController->SMx->IndexSFt; // 2*
            myCtrlThread.Des_RelativeVelocity(0) = 0.00;
            myCtrlThread.Des_RelativeVelocity(1) = 0.00;
            myCtrlThread.Des_RelativeVelocity(2) = 0.00;
            cout << " SamplingTime : "<< myCtrlThread.Parameters->SamplingTime << endl;
            cout << " Alignment bool : "<< (done) << endl;
        }

        if (done){
            myCtrlThread.Des_RelativeVelocity(0) = 0.00;
            myCtrlThread.Des_RelativeVelocity(1) = 0.00;
            myCtrlThread.Des_RelativeVelocity(2) = 0.00;
        }
        if (done && ((Time::now()-FinalMoveTime)>= (n_Samp_init *myCtrlThread.Parameters->SamplingTime))){
            finalConf = true;
            cout << " Feet Alignment done, Now closing " << endl;
        }    

    }

    // Close file
//    measurements_file.close();

    // Set to Zero the Desired CoM velocity before stopping
    myCtrlThread.Des_RelativeVelocity(0) = 0.00;  
    myCtrlThread.Des_RelativeVelocity(1) = 0.00;
    myCtrlThread.Des_RelativeVelocity(2) = 0.00;

    // Close control thread    
    myCtrlThread.stop();

    // Close Desired CoM velocity reader
    myDesiredCoM.stop();

    // close the wholebodyInterface object (robot)
    if (robot) {
        robot->close();
        delete robot;
        robot = 0;
    }

    return 0;
}
