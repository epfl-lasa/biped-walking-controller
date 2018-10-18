#ifndef CpBalWlkCtrlThread_H
#define CpBalWlkCtrlThread_H


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

#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <wbi/wbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

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

#include <qpOASES.hpp>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;

using namespace std;
using namespace Eigen;



class CpBalWlkCtrlThread : public RateThread
{
     


public:

    std::string moduleName;
    std::string robotName;
    int ThreadPeriod;
    int ForceFeedbackType;

    wbi::wholeBodyInterface& robot_model;
    wbi::Frame m_world2BaseFrame;
    Eigen::VectorXd m_world2BaseFrameSerialization;
    Eigen::VectorXd CoM_Pose;

    Eigen::VectorXd jts_position;           // q
    Eigen::VectorXd jts_velocity;           // dq
    Eigen::VectorXd jts_acceleration;       // ddq

    CpReactiveWalkingController       *CpBalWlkController;

    // internal number of cycles indicator
    int CycleCounter;
    int SwitchCounter;
    // starting time
    double start_time;

    // ----------------------------
    ControlledDevices   *RobotDevices;

    // command sender 
    CommandsWriter      CmdsWriter;
    // class gathering IMU and Torque/Firces sensors of the robots
    RobotSensors        BotSensors;

    // create an object for the kinematics of the 
    RobotKinematics     *RobotKin;

    // Posture of the robot
    RobotPostures       CtrlPostures;

    // StatesToInputCompensator
    StatesToInputCompensator    St2InCompensator;

    //
    VectorXd   OutputLegs_joints;

    InitBalWlkParameters            *Parameters;
    VectorXd               Des_RelativeVelocity;
    VectorXd               Feedback_RelativeVelocity;

    // Declaring kinematic chain of the arms force/torque sensors
    iCub::iKin::iKinChain *left_elbow_chain;
    iCub::iKin::iKinChain *right_elbow_chain;

    yarp::sig::Vector left_subchain_jts;
    yarp::sig::Vector right_subchain_jts;


    // // ports of CoM and IMU
    Bottle *CoM_values;

    BufferedPort<Bottle> CoM_port_In;
   
    ReferencesCompensator TrajectCompensation;

    //Inertial and CoM
    //VectorXd Inertial_measurements;
    VectorXd CoM_measurements;    		// as class member
    VectorXd CoM_measurements_offset;   // as class member

    VectorXd meas_acceleration_B;
    VectorXd meas_velocity_B;

    VectorXd Delta_lljoints;
    VectorXd Delta_rljoints;
    VectorXd feature_error;

    VectorXd CoM_PositionError;
    VectorXd CoM_VelocityError;
    VectorXd ModelStatesErrorX;
    VectorXd ModelStatesErrorY;
    VectorXd ModelStatesErrorR;

    // feet F/T measurements
    double foot_FT_offset_X;
    double foot_FT_offset_Y;

    VectorXd l_arm_FT_vector;
    VectorXd r_arm_FT_vector;

    double arm_FT_offset_X;
    double arm_FT_offset_Y;


    bool left_stance;
    bool writeCommands;

    firstOrderIntegrator *FilterStatesX;
    firstOrderIntegrator *FilterStatesY;
    firstOrderIntegrator *FilterStatesR;
    firstOrderIntegrator *FilterCoM_Base;
    firstOrderIntegrator *Filter_FT_LeftFoot;
    firstOrderIntegrator *Filter_FT_RightFoot;
    firstOrderIntegrator *Filter_FT_LeftArm;
    firstOrderIntegrator *Filter_FT_RightArm;
    firstOrderIntegrator *Filter_CoM_Velo;
    firstOrderIntegrator *Filter_CoM_Velo2;


    firstOrderIntegrator *Filter_CompFT_LeftArm;
    firstOrderIntegrator *Filter_CompFT_RightArm;

    KF_Velo_mAccelCldNoise *KF_VeloEstimator;

    KF_Pos_mVeloWhtNoise *KF_VeloEstimatorX;
    KF_Pos_mVeloWhtNoise *KF_VeloEstimatorY;
    KF_Pos_mVeloWhtNoise *KF_VeloEstimatorR;

    firstOrderIntegrator *Filter_IMU_RPY;

    // transformation to horizontal (IMU) frame
    // CpDesiredFeetTransformations *GaitInIMU;
    InertialCompensator  *GaitInIMU;

    // Ports to sends commands
//    BufferedPort<yarp::sig::Vector> Joints_ouput_Port;
    Port Joints_ouput_Port;
    BufferedPort<yarp::sig::Vector> CoM_output_Port;

    Vector Desired_JointsPositions;

    Vector joints_Offset;
    Vector joints_Offset_left;
    Vector joints_Offset_right;

  
    // Data_logger
    Data_logging DataLogger;

    // Smoothing of direct commands
    yarp::sig::Vector previous_commands_left_leg;
    yarp::sig::Vector previous_commands_right_leg;

    CubicInterpolator CmdSmoother;

    bool StopCtrl;

    // force/velocity dynamics
    DynamicSystemSolver X_motionDyn;
    DynamicSystemSolver Y_motionDyn;
    DynamicSystemSolver R_motionDyn;

    KinConverter YarpEigenConv;


    // Usefull Transformation
    Transformations Trnsfrms;

    // Create a object for the grasping task
    Grasping Graspingtask;


    bool StanceLeg;

    Eigen::VectorXd q_dot_lleg;
    Eigen::VectorXd q_dot_rleg;

    firstOrderIntegrator *Filter_lleg_jtsVelo;
    firstOrderIntegrator *Filter_rleg_jtsVelo;


    VectorXd qDot_Lhand_Comp;
    VectorXd qDot_Rhand_Comp;

    VectorXd tau_Lhand_Comp;
    VectorXd tau_Rhand_Comp;

    // ---------------------------------------
    // ---------------------------------------

    MatrixXd HTrsf_root_2_World;

    // ---------------------------------------

    KinConverter Pose2Matrix;

    // dynamic filter for the pattern generator
    ZmpDynamicFilter DynFilterZmpCoM;

    // Wrench transformation
    WrenchTransformation WrenchTransform;

    MatrixXd MxWrenchFoot2GF;
    //
    // +++++++++++++++++++++++++++++++++++++
    VectorXd leftArmFT_offset_sensor;
    VectorXd rightArmFT_offset_sensor;

    DynamicSystemSolver2 LShoulderPitchFilter;

    // variable to pause the walking
    bool PauseWalking;

    




    CpBalWlkCtrlThread(int period, std::string _moduleName, std::string _robotName, int FeedbackType, wbi::wholeBodyInterface& robot);
    ~CpBalWlkCtrlThread();

    bool threadInit();
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void threadRelease();
    

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Run method
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void run();
    

};

#endif //CpBalWlkCtrlThread_H