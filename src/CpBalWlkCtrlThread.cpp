



#include "CpBalWlkCtrlThread.h"


using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;

using namespace std;
using namespace Eigen;



int mysign(double Val)
{
    return (0.0 < Val) - (Val < 0.0);
}


CpBalWlkCtrlThread::CpBalWlkCtrlThread(int period, std::string _moduleName, std::string _robotName, int ForceFeedback, bool ActiveKeyBoardCtrl_, wbi::wholeBodyInterface& robot)
													: RateThread(period)
													, moduleName(_moduleName)
													, robotName(_robotName)
                                                    , robot_model(robot)
				                                    , Parameters(0)
                                                    , FilterCoM_Base(0)
                                                    , Filter_IMU_RPY(0)
				                                    , Filter_FT_LeftFoot(0)
				                                    , Filter_FT_RightFoot(0)
                                                    , RobotDevices(0)
                                                    , RobotKin(0)
				                                    , CpBalWlkController(0)
                                                    , Des_RelativeVelocity(3)
                                                    , Feedback_RelativeVelocity(3)
                                                    , m_world2BaseFrameSerialization(16)
                                                    , KeyboardCtrl(ActiveKeyBoardCtrl_)
				                                    , StopCtrl(false) 	{ 	ThreadPeriod = period; 
				                                    						ForceFeedbackType = ForceFeedback; 
				                                    					}

CpBalWlkCtrlThread::~CpBalWlkCtrlThread() {}

bool CpBalWlkCtrlThread::threadInit()
{
    //initialize here variables
    printf("CpBalanceWalkingController:starting\n");

    // *******************************************************************
    // initial parameters
    Parameters = new InitBalWlkParameters(ThreadPeriod, robotName, ForceFeedbackType);

    // initialization of the counter
    CycleCounter  = 0;

    // Initialization of the Des_RelativeVelocity
    Des_RelativeVelocity.setZero();
    Feedback_RelativeVelocity.setZero();

    int robotDof = robot_model.getDoFs();

    jts_position.resize(robotDof);           // q
    jts_velocity.resize(robotDof);           // dq
    jts_acceleration.resize(robotDof);       // ddq
    CoM_Pose.resize(7);
    CoM_measurements.resize(3);
    CoM_measurements_offset.resize(3);

    //
    // Varibale to pause the walking
    PauseWalking = false;
    //
    writeCommands = Parameters->Command_exe;

    //
    start_time = yarp::os::Time::now();

    // ===================================================================

    // Create robot devices for communication
    // ---------------------------------------
    RobotDevices = new ControlledDevices();
    RobotDevices->CreateRobotDevices(robotName, moduleName);

    // Open legs devices for communication
    RobotDevices->OpenLegsDevices();
    //
    // open arms devices for communication
    RobotDevices->OpenArmsDevices();
    //
    // Open torso devices for communication
    RobotDevices->OpenTorsoDevices();
    //
    // set acceleration  and velocity parameters for the minimum jerk planner
    RobotDevices->setLegsTrajectoryParameters( 100.0, 80.0);
    RobotDevices->setArmsTrajectoryParameters( 50.0, 30.0);
    RobotDevices->setTorsoTrajectoryParameters( 50.0, 30.0);

    // Create robot devices for sensing : IMU and COM
    // -----------------------------------------------
    BotSensors.OpenRobotSensors(robotName, *RobotDevices, Parameters->SamplingTime);

    // get IMU measurements
    BotSensors.getImuOrientationValues();
    BotSensors.getImuAccelerometerValues();

    // keyboard control
    // KeyboardCtrl  = false;
    // iskeypportActive = false;

    // // Opening the port for the Keyboard input cmds  moduleName
    // std::string KeyboardOutputs_port ="/";
    //             KeyboardOutputs_port += moduleName;
    //             KeyboardOutputs_port += "/keyboardOutputs:o";

    // if(KeyboardCtrl && !iskeypportActive)
    // {
    //     KeyboardCmd_port_In.open("/KeyboardInputCmds:i");

    //     if(!Network::connect(KeyboardOutputs_port.c_str(), KeyboardCmd_port_In.getName().c_str()))
    //     {
    //         printf(" Unable to connect to the KeyboardCmdsReaderModule port");
    //         return false;
    //     }
    //     // Reading of measurement
    //     keyboardValues  = KeyboardCmd_port_In.read(); 
    //     // alpha_velo.resize(keyboardValues->size());
    //     // alpha_velo.setZero();
    //     iskeypportActive = true;
    // }    
    // alpha_velo.resize(3);
    // alpha_velo.setZero();

    // correction of joints_offset betwen the simulator and the actual robot
    joints_Offset.resize(RobotDevices->lljoints, 0.0);
    // initial pose for Walking
    // =========================
    yarp::sig::Vector Des_lljoints, Des_rljoints;
                      Des_lljoints.resize(RobotDevices->lljoints, 0.0);
                      Des_rljoints.resize(RobotDevices->rljoints, 0.0);

    // set the robot to the initial walking posture
    CtrlPostures.moveToinitialWalkingPosture(RobotDevices->iencs_left_leg, 
                                             RobotDevices->iencs_right_leg, 
                                             RobotDevices->ipos_left_leg, 
                                             RobotDevices->ipos_right_leg, joints_Offset, 
                                             Des_lljoints, Des_rljoints);
    //
    Time::delay(1.00);
    //
    RobotDevices->setLegsTrajectoryParameters( Parameters->Accel_factor, Parameters->Speed_factor);

    // ==========================================================================================================
    // Estimation of the CoM and IMU data after initial posture
    // ********************************************************************
    // Estimation of the CoM pose
    robot_model.getEstimates(wbi::ESTIMATE_BASE_POS, m_world2BaseFrameSerialization.data());
    wbi::frameFromSerialization(m_world2BaseFrameSerialization.data(), m_world2BaseFrame);

    // Estimation og the CoM pose
    robot_model.forwardKinematics(jts_position.data(), m_world2BaseFrame, wbi::iWholeBodyModel::COM_LINK_ID, CoM_Pose.data());
    CoM_measurements = CoM_Pose.segment(0, 3);
    CoM_measurements_offset = CoM_measurements;

    // get IMU measurements
    BotSensors.getImuOrientationAcceleration();

    // get the left and right force/torque values
    BotSensors.getLeftLegForceTorqueValues();
    BotSensors.getRightLegForceTorqueValues();

    // Low pass filtering of the measurements
    FilterCoM_Base      = new firstOrderIntegrator(Parameters->SamplingTime, 3.5, 3.5, CoM_measurements); 
    Filter_IMU_RPY      = new firstOrderIntegrator(Parameters->SamplingTime, 8.5, 8.5, BotSensors.Inertial_measurements.segment(0,3));
    Filter_FT_LeftFoot  = new firstOrderIntegrator(Parameters->SamplingTime, 4., 4., BotSensors.l_foot_FT_vector);  
    Filter_FT_RightFoot = new firstOrderIntegrator(Parameters->SamplingTime, 4., 4., BotSensors.r_foot_FT_vector);

    // getting actual joints values
    RobotDevices->iencs_left_leg->getEncoders(RobotDevices->encoders_left_leg.data());
    RobotDevices->iencs_right_leg->getEncoders(RobotDevices->encoders_right_leg.data());
    RobotDevices->iencs_left_arm->getEncoders(RobotDevices->encoders_left_arm.data());
    RobotDevices->iencs_right_arm->getEncoders(RobotDevices->encoders_right_arm.data());

        
    // *******************************************************************************************
    // Kinematic model of the robot
    // *******************************************************************************************

    RobotKin  = new RobotKinematics(*RobotDevices);
    RobotKin->Initialize(*RobotDevices, Parameters->SamplingTime);

    RobotKin->UpdateLeftLegKinChain(RobotDevices->encoders_left_leg);
    RobotKin->UpdateRightLegKinChain(RobotDevices->encoders_right_leg);
    RobotKin->UpdateLeftArmKinChain(RobotDevices->encoders_left_arm);
    RobotKin->UpdateRightArmKinChain(RobotDevices->encoders_right_arm);

    // inverse kinematics with IPOPT
    // RobotKin->InitLeftLegInvKin();
    // RobotKin->InitRightLegInvKin();
    // inverse kinematics iterative gradient based
    InvKinSolver_lleg.InitializeIK(RobotKin->LeftLegChain,  0.90, 20, 1e-4, 0.450);
    InvKinSolver_rleg.InitializeIK(RobotKin->RightLegChain, 0.90, 20, 1e-4, 0.450);

    // *******************************************************************************************
    // Instantiation of the robot locomotion module
    // *******************************************************************************************
    CpBalWlkController = new CpReactiveWalkingController();
    // initialization of the controller
    CpBalWlkController->InitializeCpBalWlkController(Parameters);

    // Expression of the legs transformation with respect to the measeured horizontal plan
    GaitInIMU = new InertialCompensator();
    GaitInIMU->getDesiredFeetInCorrectedBase(   Parameters->StanceIndicator, 
                                                RobotKin->LeftLegChain->EndEffPose(), 
                                                RobotKin->RightLegChain->EndEffPose(),
                                                BotSensors.m_orientation_rpy,  
                                                CpBalWlkController->GaitTrsf);

    // *******************************************************************************************
    // Instantiation of the robot manipulation module
    // *******************************************************************************************
    Graspingtask.Inititialize(RobotKin->Left_Arm_Chain, RobotKin->Right_Arm_Chain);
    //
    Graspingtask.set_lh_GraspGain(Parameters->Gain_hand);
    Graspingtask.set_rh_GraspGain(Parameters->Gain_hand);
    //
    if(Parameters->ExecuteGrasping)
    {
        if(!Parameters->VelocityIK)
        {
            // // inverse kinematics of the arms
            // RobotKin->InitLeftArmInvKin();
            // RobotKin->InitRightArmInvKin();         
        }

    }

    // Homogeneous transformation of the robot root frame wrt. the world frame
    HTrsf_root_2_World.resize(4,4);
    HTrsf_root_2_World.setIdentity(4,4);
    // update the homogeneous transformation of the current hands frame wrt to the desired frame
    Graspingtask.UpdateTaskTransformations( HTrsf_root_2_World,
                                            Graspingtask.Desired_Left_Arm_PoseAsAxisAngles,   
                                            Graspingtask.Desired_Right_Arm_PoseAsAxisAngles,
                                            *RobotKin,
                                            false); // true if the poses are defined wrt the world.

    //
    qDot_Lhand_Comp.resize(RobotDevices->lajoints);
    qDot_Rhand_Comp.resize(RobotDevices->rajoints);

    //
    // left hand joint velocities for the grasping task  wrt. the world frame 
    qDot_Lhand_Comp = Graspingtask.get_lh_GraspJtsVelocity_World2(  HTrsf_root_2_World, 
                                                                    Graspingtask.HTrsf_CurLh_2_DesLh, 
                                                                    RobotKin->Left_Arm_Chain, 
                                                                    RobotKin->LeftLegChain,
                                                                    RobotKin->RightLegChain, 
                                                                    RobotKin->jts_velocity_lleg, 
                                                                    RobotKin->jts_velocity_rleg, 
                                                                    StanceLeg); 

    // right hand joint velocities for the grasping task  wrt. the world frame 
    qDot_Rhand_Comp = Graspingtask.get_rh_GraspJtsVelocity_World2(  HTrsf_root_2_World, 
                                                                    Graspingtask.HTrsf_CurRh_2_DesRh, 
                                                                    RobotKin->Right_Arm_Chain, 
                                                                    RobotKin->LeftLegChain, 
                                                                    RobotKin->RightLegChain, 
                                                                    RobotKin->jts_velocity_lleg, 
                                                                    RobotKin->jts_velocity_rleg,
                                                                    StanceLeg);

    // *******************************************************************************************
    // State to input compensation
    // *******************************************************************************************
    St2InCompensator.InitS2ICompensator(Parameters->SamplingTime, Parameters->VeloFeedbackThreshold);
    //
    // getting the force torque sensor offset 
    BotSensors.get_ArmsForceTorqueOffsets();

    // Compute the initial state to inout feedback
    St2InCompensator.ComputeS2IFeedback(Parameters->FT_feedback, 
                                        BotSensors.l_arm_FT_vector, 
                                        BotSensors.r_arm_FT_vector, 
                                        BotSensors.Arms_ForceTorqueOffset);

    // *******************************************************************************************
    // Initializing the command writer
    // *******************************************************************************************
    CmdsWriter.InitCommendWriter(Parameters, *RobotDevices);
    //
    // set the joints to an initial pose (TO DO)
    RobotKin->DesiredLeftLeg_joints  = RobotKin->LeftLegChain->getAng();
    RobotKin->DesiredRightLeg_joints = RobotKin->RightLegChain->getAng();
    
    RobotDevices->commands_left_leg  = RobotDevices->encoders_left_leg;
    RobotDevices->commands_right_leg = RobotDevices->encoders_right_leg;
    // set the joints to an initial pose (TO DO)
    RobotKin->DesiredLeftArm_joints  = RobotKin->Left_Arm_Chain->getAng();
    RobotKin->DesiredRightArm_joints = RobotKin->Right_Arm_Chain->getAng();
    
    RobotDevices->commands_left_arm  = RobotDevices->encoders_left_arm;
    RobotDevices->commands_right_arm = RobotDevices->encoders_right_arm;

 //=====================================================================================================
    //
    // Configuration of output ports for torqueBalancing
    // ==========================================================
    // Set impedance
    double Kp, Kv;
    Kp = 100;  // 30
    Kv = 0.707*2. * sqrt(Kp);

    if (false)
    {
        RobotDevices->iimp_left_leg->setImpedance(0, Kp, Kv);
        RobotDevices->iimp_left_leg->setImpedance(3, Kp, Kv);
        //RobotDevices->iimp_left_leg->setImpedance(4, Kp, Kv);
        // Set impedance
        RobotDevices->iimp_right_leg->setImpedance(0, Kp, Kv);
        RobotDevices->iimp_right_leg->setImpedance(3, Kp, Kv);
        //RobotDevices->iimp_right_leg->setImpedance(4, Kp, Kv);

        RobotDevices->iint_left_leg->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        RobotDevices->iint_left_leg->setInteractionMode(3, VOCAB_IM_COMPLIANT);
        //RobotDevices->iint_left_leg->setInteractionMode(4, VOCAB_IM_COMPLIANT);

        RobotDevices->iint_right_leg->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        RobotDevices->iint_right_leg->setInteractionMode(3, VOCAB_IM_COMPLIANT);
        //RobotDevices->iint_right_leg->setInteractionMode(4, VOCAB_IM_COMPLIANT);

    }

    Time::delay(1.00);

    // ==========================================================================================================            
    for (int i=0; i<RobotDevices->lljoints; i++)
    {
        // RobotDevices->ictrl_left_leg->setPositionMode(i);
        // RobotDevices->ictrl_right_leg->setPositionMode(i);
        RobotDevices->ictrl_left_leg->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
        RobotDevices->ictrl_right_leg->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
    }
    // ************************************************************
    for (int i=0; i<RobotDevices->lajoints; i++)
    {                       
        RobotDevices->ictrl_left_arm->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
        RobotDevices->ictrl_right_arm->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
    }

    // for (int i=0; i<RobotDevices->lajoints; i++)
    // {                       
    //     RobotDevices->ictrl_left_arm->setControlMode(i, VOCAB_CM_VELOCITY);
    //     RobotDevices->ictrl_right_arm->setControlMode(i, VOCAB_CM_VELOCITY);
    // }

    // Impedance mode
        Kp = 100;  // 
    //Kv = 20.0;
    Kv = 0.707*0.25 * sqrt(Kp);

    if (false)
    {
    for (int i=0; i<RobotDevices->lajoints; i++)
    {                       
        RobotDevices->iimp_left_arm->setImpedance(i, Kp, Kv);
        RobotDevices->iimp_right_arm->setImpedance(i, Kp, Kv);
    }
    }
    // Torque control mode
    for (int i=0; i<RobotDevices->lajoints; i++)
    {                       
        // RobotDevices->ictrl_left_arm->setControlMode(i, VOCAB_CM_TORQUE);
        // RobotDevices->ictrl_right_arm->setControlMode(i, VOCAB_CM_TORQUE);
        // RobotDevices->iint_left_arm->setInteractionMode(i, VOCAB_IM_COMPLIANT);
        // RobotDevices->iint_right_arm->setInteractionMode(i, VOCAB_IM_COMPLIANT);
        RobotDevices->iint_left_arm->setInteractionMode(i, VOCAB_IM_STIFF);
        RobotDevices->iint_right_arm->setInteractionMode(i, VOCAB_IM_STIFF);
    }
     
    // for (int i=0; i<RobotDevices->lajoints; i++)
    // {                       
    //     RobotDevices->ictrl_left_arm->setControlMode(i, VOCAB_CM_POSITION);
    //     RobotDevices->ictrl_right_arm->setControlMode(i, VOCAB_CM_POSITION);
    // }


    // ==================================================================================
    VectorXd Test_Data0(2);
    Test_Data0.setZero(2);


    // Logging the data
    DataLogger.InitializeLogger();

    DataLogger.Write_Data(Parameters->SamplingTime,
                          CycleCounter,
                          CpBalWlkController->CoPref,
                          CpBalWlkController->DMod,
                          CpBalWlkController->FtTraj,
                          CpBalWlkController->VeloRef,
                          0.*RobotDevices->commands_left_leg,
                          0.*RobotDevices->commands_right_leg,
                          0.*RobotDevices->encoders_left_leg,
                          0.*RobotDevices->encoders_right_leg,
                          BotSensors.l_foot_FT_vector,
                          BotSensors.r_foot_FT_vector,
                          CoM_measurements,
                          Test_Data0);


    return true;
}
// ============================================================================================================================


// ============================================================================================================================



void CpBalWlkCtrlThread::threadRelease()
{
    printf("CpBalanceWalkingController:stopping the robot\n");

    RobotDevices->iint_right_leg->setInteractionMode(0, VOCAB_IM_STIFF);
    RobotDevices->iint_right_leg->setInteractionMode(3, VOCAB_IM_STIFF);
    RobotDevices->iint_right_leg->setInteractionMode(4, VOCAB_IM_STIFF);                


    // set back to position control mode
    for (int i=0; i<RobotDevices->lljoints; i++)
    {
        RobotDevices->ictrl_left_leg->setControlMode(i, VOCAB_CM_POSITION);
        RobotDevices->ictrl_right_leg->setControlMode(i, VOCAB_CM_POSITION);
    }

    // 
    for (int i=0; i<RobotDevices->lajoints; i++)
    {                       
        RobotDevices->ictrl_left_arm->setControlMode(i, VOCAB_CM_POSITION);
        RobotDevices->ictrl_right_arm->setControlMode(i, VOCAB_CM_POSITION);
    }


    Time::delay(0.04);


    // left
    for (int i = 0; i < RobotDevices->lljoints; i++) {
        RobotDevices->commands_left_leg[i] = 100.0;
    }
    RobotDevices->ipos_left_leg->setRefAccelerations(RobotDevices->commands_left_leg.data());

    for (int i = 0; i < RobotDevices->lljoints; i++) {
        RobotDevices->commands_left_leg[i] = 80.0;
        RobotDevices->ipos_left_leg->setRefSpeed(i, RobotDevices->commands_left_leg[i]);
    }

    // right
    for (int i = 0; i < RobotDevices->rljoints; i++) {
        RobotDevices->commands_right_leg[i] = 100.0;
    }
    RobotDevices->ipos_right_leg->setRefAccelerations(RobotDevices->commands_right_leg.data());

    for (int i = 0; i < RobotDevices->rljoints; i++) {
        RobotDevices->commands_right_leg[i] = 80.0;
        RobotDevices->ipos_right_leg->setRefSpeed(i, RobotDevices->commands_right_leg[i]);
    }

    yarp::sig::Vector Des_lljoints, Des_rljoints;
                      Des_lljoints.resize(RobotDevices->lljoints, 0.0);
                      Des_rljoints.resize(RobotDevices->rljoints, 0.0);

   CtrlPostures.moveTofinalWalkingPosture( RobotDevices->iencs_left_leg, RobotDevices->iencs_right_leg, RobotDevices->ipos_left_leg, RobotDevices->ipos_right_leg, joints_Offset, Des_lljoints, Des_rljoints);


    //
    RobotDevices->CloseLegsDevices();
    RobotDevices->CloseArmsDevices();
    RobotDevices->CloseTorsoDevices();

    BotSensors.CloseRobotSensors();

    // close the data Logger
    DataLogger.Close_files();

    // closing the keyboard reader
    // if(KeyboardCtrl)
    // {
    //     KeyboardCmd_port_In.close();
    // }
    

    // release the CpWalkingController
    CpBalWlkController->ReleaseCpBalWlkController();

    if (CpBalWlkController)
    {
        delete CpBalWlkController;
        CpBalWlkController = 0;
    }

    

// ----------------------------

    if (Parameters) {
        delete Parameters;
        Parameters = 0;
    }

    if (RobotDevices) {
        delete RobotDevices;
        RobotDevices = 0;
    }

    if (RobotKin) {
        delete RobotKin;
        RobotKin = 0;
    }
    

    if (FilterCoM_Base)
    {
        delete FilterCoM_Base;
        FilterCoM_Base = 0;
    }

    if (Filter_FT_LeftFoot) {
        delete Filter_FT_LeftFoot;
        Filter_FT_LeftFoot = 0;
    }

    if (Filter_FT_RightFoot) {
        delete Filter_FT_RightFoot;
        Filter_FT_RightFoot = 0;
    }

    if (GaitInIMU)
    {
        delete GaitInIMU;
        GaitInIMU = 0;
    }

    if (Filter_IMU_RPY)
    {
        delete Filter_IMU_RPY;
        Filter_IMU_RPY = 0;
    }

    if (Filter_CompFT_LeftArm)
    {
        delete Filter_CompFT_LeftArm;
        Filter_CompFT_LeftArm = 0;
    }

    if (Filter_CompFT_RightArm)
    {
        delete Filter_CompFT_RightArm;
        Filter_CompFT_RightArm = 0;
    }

 
cout << " Done 2" << endl;

    printf("Done, goodbye from CpBalWlkCtrlThread\n");
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Run method
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CpBalWlkCtrlThread::run()
{
    if (CycleCounter == 0){

        printf("CpBalWlkCtrlThread: now running \n");
    }

    cout << "Iteration : " << CycleCounter +1 << endl;


    // -------------------------------------------------------------------
    double t_run = Time::now();
    printf("Desired Relative Velocity  vx:%4.6f vy:%4.6f  wz:%4.6f \n", Des_RelativeVelocity(0), Des_RelativeVelocity(1), Des_RelativeVelocity(2));
    printf("Current Robot COM Position x:%4.6f y:%4.6f  z:%4.6f \n", CoM_measurements(0), CoM_measurements(1), CoM_measurements(2));

    CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity+Feedback_RelativeVelocity, CycleCounter);

    if (CycleCounter == 0)
    {
       //TrajectCompensation.PrevStanceY = CpBalWlkController->CoPref->CoPRefY;
    }

    // *****************************************************************************************
    // Estimation of the robot state
    // *****************************************************************************************
    // get encoders values and update the chains
    RobotDevices->iencs_left_leg->getEncoders(RobotDevices->encoders_left_leg.data());
    RobotDevices->iencs_right_leg->getEncoders(RobotDevices->encoders_right_leg.data());
    //
    RobotDevices->iencs_left_arm->getEncoders(RobotDevices->encoders_left_arm.data());
    RobotDevices->iencs_right_arm->getEncoders(RobotDevices->encoders_right_arm.data());
    //
    RobotKin->UpdateLeftLegKinChain(RobotDevices->encoders_left_leg);
    RobotKin->UpdateRightLegKinChain(RobotDevices->encoders_right_leg);
    //
    RobotKin->UpdateLeftArmKinChain(RobotDevices->encoders_left_arm);
    RobotKin->UpdateRightArmKinChain(RobotDevices->encoders_right_arm);

    // get CoM measurements

    // get IMU measurements
    BotSensors.getImuOrientationAcceleration();
        
    // Feet Force/Torque measurement
    BotSensors.getLeftLegForceTorqueValues();
    BotSensors.getRightLegForceTorqueValues();

    BotSensors.l_foot_FT_vector = Filter_FT_LeftFoot->getRK4Integral(BotSensors.l_foot_FT_vector);
    BotSensors.r_foot_FT_vector = Filter_FT_RightFoot->getRK4Integral(BotSensors.r_foot_FT_vector);
    // Compute the wrench with respect to the world foot frame (with the x axis pointing in front of the robot and the z axis upright)
    // BotSensors.l_foot_FT_vector = MxWrenchFoot2GF * Filter_FT_LeftFoot->getRK4Integral(BotSensors.l_foot_FT_vector);
    // BotSensors.r_foot_FT_vector = MxWrenchFoot2GF * Filter_FT_RightFoot->getRK4Integral(BotSensors.r_foot_FT_vector);

    // update the stance leg indicator
    // ---------------------------------

    // Compensation for the ground orientation (inclination of the ground)
    // -------------------------------------------------------------------
    Eigen::VectorXd IMU_RYP = Filter_IMU_RPY->getRK4Integral(BotSensors.Inertial_measurements.segment(0,3));

    GaitInIMU->CompensateTransformsWithIMU( CpBalWlkController->GaitTrsf,
                                            RobotKin->LeftLegChain->EndEffPose(), 
                                            RobotKin->RightLegChain->EndEffPose(),                                                      
                                            BotSensors.m_orientation_rpy,  // filtered
                                            Parameters->StanceIndicator,
                                            Parameters->IMU_reference);

    // ***************************************************************************************
    // INVERSE KINEMATICS OF THE DESIRED LEGS CONFIGURATION
    // *************************************************************************************** 

    double t_IK =Time::now();

    // RobotKin->ComputeLeftLegInvKin(GaitInIMU->DesiredLeftLegPoseAsAxisAngles, RobotDevices->encoders_left_leg, true);
    // RobotKin->ComputeRightLegInvKin(GaitInIMU->DesiredRightLegPoseAsAxisAngles, RobotDevices->encoders_right_leg, true);
    yarp::sig::Vector myDes_left_leg_joints  = InvKinSolver_lleg.get_IKsolution(GaitInIMU->DesiredLeftLegPoseAsAxisAngles, RobotDevices->encoders_left_leg, true);
    yarp::sig::Vector myDes_right_leg_joints = InvKinSolver_rleg.get_IKsolution(GaitInIMU->DesiredRightLegPoseAsAxisAngles, RobotDevices->encoders_right_leg, true);

    // // compute the arms inverse kinematics
    // RobotKin->ComputeLeftArmInvKin(Graspingtask.Desired_Left_Arm_PoseAsAxisAngles, RobotDevices->encoders_left_arm, true);
    // RobotKin->ComputeRightArmInvKin(Graspingtask.Desired_Right_Arm_PoseAsAxisAngles, RobotDevices->encoders_right_arm, true);  

    // ============  ADDING JOINTS COMPENSATION ===============================
          
    // ============  SENDING THE COMMANDS TO THE JOINTS =======================
    double t_IK_0=Time::now();

    if (writeCommands)
    {
        //
        //CmdsWriter.WriteLegsCommands(RobotKin->DesiredLeftLeg_joints, RobotKin->DesiredRightLeg_joints, *RobotDevices);
        CmdsWriter.WriteLegsCommands(myDes_left_leg_joints, myDes_right_leg_joints, *RobotDevices);
    }

    double t_IK_f = Time::now();

    // modifying the desired pose
    double ref_period = 3.0;
    double omega = 2.0*M_PI/ref_period;
    //
    Graspingtask.Desired_Left_Arm_PoseAsAxisAngles[1]  = Graspingtask.desired_lhand_pose_in_root[1] + 0.0 * sin(omega * (yarp::os::Time::now() - start_time));
    Graspingtask.Desired_Right_Arm_PoseAsAxisAngles[1] = Graspingtask.desired_rhand_pose_in_root[1] + 0.0 * sin(omega * (yarp::os::Time::now() - start_time));

    
    if(Parameters->ExecuteGrasping)
    {
        if(!Parameters->VelocityIK)
        {
            // // compute the arms inverse kinematics
            // RobotKin->ComputeLeftArmInvKin(Graspingtask.Desired_Left_Arm_PoseAsAxisAngles, RobotDevices->encoders_left_arm, true);
            // RobotKin->ComputeRightArmInvKin(Graspingtask.Desired_Right_Arm_PoseAsAxisAngles, RobotDevices->encoders_right_arm, true);
            // // command writer
            // CmdsWriter.WriteArmsCommands(RobotKin->DesiredLeftArm_joints, RobotKin->DesiredRightArm_joints, *RobotDevices); 

        }
        else  // for the velocity based IK
        {
            // get the transformation of the root wrt the world
            HTrsf_root_2_World = HTrsf_root_2_World;  // normally from exteroception

            // update the homogeneous transformation of the current hands frame wrt to the desired frame
            Graspingtask.UpdateTaskTransformations( HTrsf_root_2_World,
                                                    Graspingtask.Desired_Left_Arm_PoseAsAxisAngles,   
                                                    Graspingtask.Desired_Right_Arm_PoseAsAxisAngles,
                                                    *RobotKin,
                                                    false); // true if the poses are defined wrt the world.

            // Update the legs joints states (to approximate the velocities)
            RobotKin->UpdateLegsJointsStates(   RobotDevices->encoders_left_leg,
                                                RobotDevices->encoders_left_leg,
                                                Parameters->SamplingTime,
                                                true);

            // determine the stance leg
                if (Parameters->StanceIndicator[0] == 1){
                    StanceLeg = true;
                }
                else {
                    StanceLeg = false;
                }


            // left hand joint velocities for the grasping task  wrt. the world frame 
            qDot_Lhand_Comp = Graspingtask.get_lh_GraspJtsVelocity_World2(  HTrsf_root_2_World, 
                                                                            Graspingtask.HTrsf_CurLh_2_DesLh, 
                                                                            RobotKin->Left_Arm_Chain, 
                                                                            RobotKin->LeftLegChain,
                                                                            RobotKin->RightLegChain, 
                                                                            RobotKin->jts_velocity_lleg, 
                                                                            RobotKin->jts_velocity_rleg, 
                                                                            StanceLeg); 

            // // right hand joint velocities for the grasping task  wrt. the world frame 
            qDot_Rhand_Comp = Graspingtask.get_rh_GraspJtsVelocity_World2(  HTrsf_root_2_World, 
                                                                            Graspingtask.HTrsf_CurRh_2_DesRh, 
                                                                            RobotKin->Right_Arm_Chain, 
                                                                            RobotKin->LeftLegChain, 
                                                                            RobotKin->RightLegChain, 
                                                                            RobotKin->jts_velocity_lleg, 
                                                                            RobotKin->jts_velocity_rleg,
                                                                            StanceLeg);

            // // left hand joint velocities for the grasping task  wrt. the world frame 
            // qDot_Lhand_Comp = Graspingtask.GetQPsolution_lh_GraspWorld2(  HTrsf_root_2_World, 
            //                                                                 Graspingtask.HTrsf_CurLh_2_DesLh, 
            //                                                                 RobotKin->Left_Arm_Chain, 
            //                                                                 RobotKin->LeftLegChain,
            //                                                                 RobotKin->RightLegChain, 
            //                                                                 RobotKin->jts_velocity_lleg, 
            //                                                                 RobotKin->jts_velocity_rleg, 
            //                                                                 StanceLeg,
            //                                                                 Parameters->SamplingTime,
            //                                                                 Graspingtask.velocity_Saturation_Limit); 

            // // right hand joint velocities for the grasping task  wrt. the world frame 
            // qDot_Rhand_Comp = Graspingtask.GetQPsolution_rh_GraspWorld2(  HTrsf_root_2_World, 
            //                                                                 Graspingtask.HTrsf_CurRh_2_DesRh, 
            //                                                                 RobotKin->Right_Arm_Chain, 
            //                                                                 RobotKin->LeftLegChain, 
            //                                                                 RobotKin->RightLegChain, 
            //                                                                 RobotKin->jts_velocity_lleg, 
            //                                                                 RobotKin->jts_velocity_rleg,
            //                                                                 StanceLeg,
            //                                                                 Parameters->SamplingTime,
            //                                                                 Graspingtask.velocity_Saturation_Limit);

            // command writer
            // CmdsWriter.WriteArmsVelocityCommands( qDot_Lhand_Comp, qDot_Rhand_Comp, *RobotDevices);
            yarp::sig::Vector Des_left_arm_jts_cmd(RobotDevices->lajoints); 
            yarp::sig::Vector Des_right_arm_jts_cmd(RobotDevices->rajoints); 

            for(int i=0; i<RobotDevices->lajoints; i++ )
            {
                Des_left_arm_jts_cmd[i]  = CTRL_DEG2RAD *RobotDevices->encoders_left_arm[i] + (Parameters->SamplingTime) * qDot_Lhand_Comp(i); 
                Des_right_arm_jts_cmd[i] = CTRL_DEG2RAD *RobotDevices->encoders_right_arm[i] + Parameters->SamplingTime * qDot_Rhand_Comp(i);
            }
            CmdsWriter.WriteArmsCommands(   Des_left_arm_jts_cmd, 
                                            Des_right_arm_jts_cmd, 
                                            *RobotDevices);

        }

    }
    else
    {
        // use admittance with arms force torque measurement
        // -------------------------------------------------
        BotSensors.getLeftArmForceTorqueValues();
        BotSensors.getRightArmForceTorqueValues();

        // Compute the initial state to inout feedback
        Eigen::VectorXd EstimatedFeet_FT(12);
        Eigen::VectorXd EstimatedArm_FT(12);
        EstimatedFeet_FT.head(6) = BotSensors.l_foot_FT_vector;
        EstimatedFeet_FT.tail(6) = BotSensors.r_foot_FT_vector;

        EstimatedArm_FT.head(6) = BotSensors.l_arm_FT_vector;
        EstimatedArm_FT.tail(6) = BotSensors.r_arm_FT_vector;

        St2InCompensator.ComputeS2IFeedback(Parameters->FT_feedback, 
                                            EstimatedFeet_FT, 
                                            EstimatedArm_FT, 
                                            BotSensors.Arms_ForceTorqueOffset);
        
        // assign velocity feedback
        Feedback_RelativeVelocity(0) = St2InCompensator.VxComFeedback;      // 0.5
        Feedback_RelativeVelocity(1) = St2InCompensator.VyComFeedback;      // 0.5
        Feedback_RelativeVelocity(2) = St2InCompensator.WzComFeedback;      // 0.5

    }

    // ================ TRAJECTORY TRACKING COMPENSATION =======================
    //
    yarp::sig::Vector F_lleg_jts_yarp(RobotKin->LeftLegChain->getDOF()),
                      F_rleg_jts_yarp(RobotKin->LeftLegChain->getDOF());

                      F_lleg_jts_yarp = 0.0;
                      F_rleg_jts_yarp = 0.0;
            
    // ***************************************************************************************
    // RECORDING THE DATA
    // *************************************************************************************** 

    double t_IK_write = Time::now();

    VectorXd Test_Data(2);
    Test_Data.setZero(2);
    Test_Data(0) = t_IK_f-t_run;
    Test_Data(1) = t_IK_f-t_IK_0; 

    // **************************************
    // Reading the keyboard inputs and placing in alpha_velo vector
    // if(KeyboardCtrl)
    // {
    //     keyboardValues  = KeyboardCmd_port_In.read(); 

    //     // Extract the read value
    //     alpha_velo(0) = keyboardValues->get(0).asDouble();
    //     alpha_velo(1) = keyboardValues->get(1).asDouble();
    //     alpha_velo(2) = keyboardValues->get(2).asDouble();

    // }

    DataLogger.Write_Data(Parameters->SamplingTime,
                         CycleCounter,
                         CpBalWlkController->CoPref,
                         CpBalWlkController->DMod,
                         CpBalWlkController->FtTraj,
                         CpBalWlkController->VeloRef,
                         F_lleg_jts_yarp,
                         F_rleg_jts_yarp,
                         RobotDevices->encoders_left_leg,
                         RobotDevices->encoders_right_leg,
                         BotSensors.l_foot_FT_vector,
                         BotSensors.r_foot_FT_vector,
                         CoM_measurements,
                         Test_Data);

   // 
   // stop if the robot loses contact wit the ground
    if ((CycleCounter > 2) && (fabs(BotSensors.l_foot_FT_vector(2) + BotSensors.r_foot_FT_vector(2)) < 120.0)) // 120   200
    {
        StopCtrl = true;
    }

    // **********************************************************************
    //printf(".");
    cout << "CycleCounter  " << CycleCounter << " solved in: " << Time::now()-t_run << " s " <<endl;

    // Update of the counter
    CycleCounter ++;
            
}


  
       