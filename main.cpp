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
#include "SubLimbsUpperBody.h"
#include "ArmsForceTorqueKinChain.h"
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
    rf.setDefaultConfigFile("BalanceWalkingController.ini");        //default config file name.
    //rf.setDefaultContext("wholeBodyHPIDControl");             //when no parameters are given to the module this is the default context
    rf.configure(argc, argv);
    
    if (rf.check("help")) 
    {
        std::cout<< "Possible parameters" << std::endl << std::endl;
        std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /BalanceWalkingController/config" << std::endl;
        std::cout<< "\t--rate             :Period used by the module. Default set to 10ms." << std::endl;
        std::cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default." << std::endl;
        std::cout<< "\t--moduleName       :name of the module ." << std::endl;
        std::cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyHPIDControl." << std::endl;
//        codyco::iCubPartVersionOptionsPrint();
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

    //int actuatedDOFs = robotMainJoints.size();

    //Load configuration-time parameters
    std::string m_moduleName = rf.check("name", Value("CpBalanceWalkingController"), "Looking for module name").asString();
    std::string m_robotName = rf.check("robot", Value("icubGazeboSim"), "Looking for robot name").asString();
    int period          = rf.check("period", Value(40), "Looking for controller period").asInt();
    double RunDuration   = rf.check("duration", Value(5), "Looking for Running Duration ").asDouble();
    int FT_feedbackType  = rf.check("FT_feedback", Value(0), "Looking for Running Duration ").asInt();

    // std::string robotName  = rf.find("robot").asString();
    // std::string m_moduleName = rf.find("name").asString();

    // std::cout<< "Running with:" <<std::endl;
    // std::cout<< "m_moduleName: " << m_moduleName.c_str() <<std::endl;

    // set the period of the pattern generator
    //.int period = 40; // [ms]

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

    // instantiate the 
    CpBalWlkCtrlThread myThread(period, m_moduleName, m_robotName, FT_feedbackType, *robot); //period is 40ms

    // Starting the BalanceWalkingController Thread
    myThread.start();

     // variables for end of walking configuration
    int n_Samp1, n_Samp_init;
        n_Samp1 = (int)(round(myThread.Parameters->DurationSteps[0]/myThread.Parameters->SamplingTime));

    // Set the Desired CoM velocity
    myThread.Des_RelativeVelocity(0) = 0.16;
    myThread.Des_RelativeVelocity(1) = 0.00;
    myThread.Des_RelativeVelocity(2) = 0.0;

    // Input velocity low pass Filter
    firstOrderIntegrator *FilterDesiredVelocity;

    FilterDesiredVelocity = new firstOrderIntegrator(myThread.Parameters->SamplingTime, 2.8, 2.8, myThread.Des_RelativeVelocity); 

    //bool ft_sensing = false;

    bool done=false;
    bool finalConf = false;
    double startTime=Time::now();
    double FinalMoveTime = Time::now();

    // variable for feet alignment and stopping
    bool AlignFeet = false;
    bool noInput   = false;

    // =================================

    while(!(done && finalConf) && !myThread.StopCtrl)
    //while(!done)
    {
        // filter the velocity
        myThread.Des_RelativeVelocity = FilterDesiredVelocity->getRK4Integral(myThread.Des_RelativeVelocity);

            // if (!noInput && (myThread.Des_RelativeVelocity(0)== 0.00)
            //              && (myThread.Des_RelativeVelocity(1)== 0.00)
            //              && (myThread.Des_RelativeVelocity(2)== 0.00))
            // {
            //     noInput = !noInput;  // from false to true
            //     FinalMoveTime = Time::now();

            //     n_Samp_init = 2*(n_Samp1 - 1) - myThread.CpBalWlkController->SMx->IndexSFt; // 2*

            //     myThread.Des_RelativeVelocity(0) = 0.00;
            //     myThread.Des_RelativeVelocity(1) = 0.00;
            //     myThread.Des_RelativeVelocity(2) = 0.00;

            // }
            // if (noInput)
            // {
            //     myThread.Des_RelativeVelocity(0) = 0.00;
            //     myThread.Des_RelativeVelocity(1) = 0.00;
            //     myThread.Des_RelativeVelocity(2) = 0.00;
            // }

            // if (noInput && ((Time::now()-FinalMoveTime)>= (n_Samp_init * myThread.Parameters->SamplingTime)))
            // {
            //     AlignFeet = true;
            //     cout << " Pause the walking " << endl;

            //     noInput = !noInput;

            //     myThread.PauseWalking = AlignFeet;
            // }
            // else
            // {
            //     AlignFeet = false;
            //     myThread.PauseWalking = AlignFeet;

            // }

      
            if (!done &&((Time::now()-startTime)> RunDuration))
            {

                done=true;

                FinalMoveTime = Time::now();

                n_Samp_init = 2*(n_Samp1 - 1) - myThread.CpBalWlkController->SMx->IndexSFt; // 2*

                myThread.Des_RelativeVelocity(0) = 0.00;
                myThread.Des_RelativeVelocity(1) = 0.00;
                myThread.Des_RelativeVelocity(2) = 0.00;

                cout << " SamplingTime : "<< myThread.Parameters->SamplingTime << endl;
                cout << " Alignment bool : "<< (done) << endl;

                //finalConf = true;
            }

            if (done)
            {
                myThread.Des_RelativeVelocity(0) = 0.00;
                myThread.Des_RelativeVelocity(1) = 0.00;
                myThread.Des_RelativeVelocity(2) = 0.00;
            }

            //cout << " Stopping time :\n"<< (n_Samp_init *myThread.Parameters->SamplingTime) << endl;

            if (done && ((Time::now()-FinalMoveTime)>= (n_Samp_init *myThread.Parameters->SamplingTime)))
            {
                finalConf = true;

                cout << " Feet Alignment done, Now closing " << endl;
            }

            // // filter the velocity
            // myThread.Des_RelativeVelocity = FilterDesiredVelocity->getRK4Integral(myThread.Des_RelativeVelocity);

        //Time::delay(m_period);

    }

    //endwin();


    // Set to Zero the Desired CoM velocity before stoping
    myThread.Des_RelativeVelocity(0) = 0.00;  // TO DO
    myThread.Des_RelativeVelocity(1) = 0.00;
    myThread.Des_RelativeVelocity(2) = 0.00;

    // Bring the feet at the initial configuration
    // int n_Samp1, n_Samp_init;
    
    myThread.stop();


    if (FilterDesiredVelocity)
    {
        delete FilterDesiredVelocity;
        FilterDesiredVelocity = 0;
    }

    // close the wholebodyInterface object (robot)
    if (robot) 
    {
        robot->close();
        delete robot;
        robot = 0;
    }


    return 0;
}
