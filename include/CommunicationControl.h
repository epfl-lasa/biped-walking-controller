

/* 
 * Copyright (C) 2017 Learning Algorithms and Systems Laboratory (LASA)
 * Author: Michael Bombile
 * email:  michael.bombile@epfl.ch
 * website: lasa.epfl.ch
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


// ===============================================================================================
// ControlledDevices : This class comprises 4 subclasses that ensure communication and control 
// CommunicationControl
// 	7.1. ControlledDevices
// 	7.2. RobotSensors
// 	7.3. RobotPostures
// 	7.4. SyncRobotModelStanceFoot
//  7.5. CommandsWrter
// ===============================================================================================


#ifndef CommunicationControl_H
#define CommunicationControl_H


#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl2.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>



//#include "InitParameters.h"
//#include "UtilityFunctions.h"
#include "InitBalWlkParameters.h"
#include "CpMath_Utilities.h"
//#include "PatternsGenerator.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

using namespace std;
using namespace Eigen;

// =============================================================================================
/*
 * ControlledDevices : This class comprises 4 subclasses that ensure communication and control 
 * of the robot 
 *
*/
// =============================================================================================

class ControlledDevices
{


    std::string RobotName;
    std::string ModuleName;
    
    public :

        // left leg
        PolyDriver          clientLeftLeg;
        IPositionControl    *ipos_left_leg;
        IPositionDirect     *ipos_left_legDir;
        IImpedanceControl   *iimp_left_leg;
        IControlMode2       *ictrl_left_leg;
        IInteractionMode    *iint_left_leg;
        ITorqueControl      *itrq_left_leg;
        IEncoders           *iencs_left_leg;
        IVelocityControl    *ivel_left_leg;

        // right legs       
        PolyDriver          clientRightLeg;
        IPositionControl    *ipos_right_leg;
        IPositionDirect     *ipos_right_legDir;
        IImpedanceControl   *iimp_right_leg;
        IControlMode2       *ictrl_right_leg;
        IInteractionMode    *iint_right_leg;
        ITorqueControl      *itrq_right_leg;
        IEncoders           *iencs_right_leg;
        IVelocityControl    *ivel_right_leg;

         // left arm
        PolyDriver          clientLeftArm;
        IPositionControl    *ipos_left_arm;
        IPositionDirect     *ipos_left_armDir;
        IImpedanceControl   *iimp_left_arm;
        IControlMode2       *ictrl_left_arm;
        IInteractionMode    *iint_left_arm;
        ITorqueControl      *itrq_left_arm;
        IEncoders           *iencs_left_arm;
        IVelocityControl    *ivel_left_arm;

        // right arm       
        PolyDriver          clientRightArm;
        IPositionControl    *ipos_right_arm;
        IPositionDirect     *ipos_right_armDir;
        IImpedanceControl   *iimp_right_arm;
        IControlMode2       *ictrl_right_arm;
        IInteractionMode    *iint_right_arm;
        ITorqueControl      *itrq_right_arm;
        IEncoders           *iencs_right_arm;
        IVelocityControl    *ivel_right_arm;

        // Torso     
        PolyDriver          clientTorso;
        IPositionControl    *ipos_torso;
        IPositionDirect     *ipos_torsoDir;
        IImpedanceControl   *iimp_torso;
        IControlMode2       *ictrl_torso;
        IInteractionMode    *iint_torso;
        ITorqueControl      *itrq_torso;
        IEncoders           *iencs_torso;
        IVelocityControl    *ivel_torso;

        int lljoints;
        int rljoints;
        int lajoints;
        int rajoints;
        int torsojoints;

        //
        // legs 
        yarp::sig::Vector       commands_left_leg;
        yarp::sig::Vector       commands_right_leg;
        // arms
        yarp::sig::Vector       commands_left_arm;
        yarp::sig::Vector       commands_right_arm;
        
        yarp::sig::Vector       torqueCmd_left_arm;
        yarp::sig::Vector       torqueCmd_right_arm;

        yarp::sig::Vector       VelocityCmd_left_arm;
        yarp::sig::Vector       VelocityCmd_right_arm;
        // Torso
        yarp::sig::Vector       commands_torso;

        // legs 
        yarp::sig::Vector       encoders_left_leg;
        yarp::sig::Vector       encoders_right_leg;
        // // arms
        yarp::sig::Vector       encoders_left_arm;
        yarp::sig::Vector       encoders_right_arm;
        // // Torso 
        yarp::sig::Vector       encoders_torso;


        ControlledDevices();
        
        ~ControlledDevices();

        void CreateRobotDevices(std::string robotName,
                                std::string moduleName);
       
        void OpenLegsDevices();

        void setLegsTrajectoryParameters( double AccelParam,
                                          double SpeedParam);
        
        void CloseLegsDevices();
        
        // arms
        // ~~~~~

        void OpenArmsDevices();

        void setArmsTrajectoryParameters( double AccelParam,
                                          double SpeedParam);
        
        void CloseArmsDevices();
        

        // Torso
        // ~~~~~

        void OpenTorsoDevices();

        void setTorsoTrajectoryParameters( double AccelParam,
                                           double SpeedParam);
        
        void CloseTorsoDevices();


};


// ===============================================================================================

// ===============================================================================================

/*
 * CP_QPSolver_OASES : This class encodes a QP solver based on 
 * qpOASES library 
 *
*/


class RobotSensors
{
	
	public :

		// ports of CoM and IMU
	    // Bottle *CoM_values;
	    Bottle *IMU_values;

	    // BufferedPort<Bottle> CoM_port_In;
	    BufferedPort<Bottle> IMU_port_In;

	    // Feet Force/torque sensors ports	    
	    BufferedPort<Bottle> l_foot_FT_inputPort;
	    BufferedPort<Bottle> r_foot_FT_inputPort;

	    BufferedPort<Bottle> l_arm_FT_inputPort;
	    BufferedPort<Bottle> r_arm_FT_inputPort;

	    Bottle *l_foot_FT_data;
	    Bottle *r_foot_FT_data;

	    Bottle *l_arm_FT_data;
	    Bottle *r_arm_FT_data;


	    //Inertial and CoM
	    Eigen::VectorXd   Inertial_measurements;
	    // Eigen::VectorXd   CoM_measurements;    		// as class member
	    Eigen::VectorXd   m_acceleration;
	    yarp::sig::Vector m_orientation_rpy;
	    Eigen::VectorXd   m_gyro_xyz;


	    // feet F/T measurements
	    Eigen::VectorXd l_foot_FT_vector;
	    Eigen::VectorXd r_foot_FT_vector;
        //
	    Eigen::VectorXd l_arm_FT_vector;
	    Eigen::VectorXd r_arm_FT_vector;
        //
        Eigen::Vector3d Arms_ForceTorqueOffset;
        //
        firstOrderIntegrator *Filter_FT_LeftArm;
        firstOrderIntegrator *Filter_FT_RightArm;

	    // // legs 
     //    yarp::sig::Vector       encoders_left_leg;
     //    yarp::sig::Vector       encoders_right_leg;

     //    // arms 
     //    yarp::sig::Vector       encoders_left_arm;
     //    yarp::sig::Vector       encoders_right_arm;

     //    // torso 
     //    yarp::sig::Vector       encoders_torso;
     //    // head 
     //    yarp::sig::Vector       encoders_head;




	    RobotSensors();
	    ~ RobotSensors();

	    void OpenRobotSensors(std::string robotName, ControlledDevices &botDevices, double SamplingTime);
	    void CloseRobotSensors();
	    yarp::sig::Vector getLeftLegEncodersValue(ControlledDevices &botDevices);
	    yarp::sig::Vector getRightLegEncodersValue(ControlledDevices &botDevices);
	    yarp::sig::Vector getLeftArmEncodersValue(ControlledDevices &botDevices);
	    yarp::sig::Vector getRightArmEncodersValue(ControlledDevices &botDevices);
	    yarp::sig::Vector getTorsoEncodersValue(ControlledDevices &botDevices);

	    // Eigen::VectorXd getCoMValues();
	    yarp::sig::Vector getImuOrientationValues();
	    Eigen::VectorXd getImuAccelerometerValues();
	    Eigen::VectorXd getImuGyroValues();
        bool getImuOrientationAcceleration();
	    Eigen::VectorXd getLeftArmForceTorqueValues();
	    Eigen::VectorXd getRightArmForceTorqueValues();
	    Eigen::VectorXd getLeftLegForceTorqueValues();
	    Eigen::VectorXd getRightLegForceTorqueValues();
        // 
        void get_ArmsForceTorqueOffsets();

};

// // ===============================================================================================

// // ===============================================================================================


// /*
//  * CP_QPSolver_OASES : This class encodes a QP solver based on 
//  * qpOASES library 
//  *
// */

class RobotPostures
{
    

    
        VectorXd TrslBaseCoM;      // Translation vector from base to CoM

        public :

        MatrixXd DesTrsfRealStanceFootInHorBase;
        MatrixXd CurTrsfRealStanceFootInHorBase;
         
        // Methods
          
        RobotPostures();

        ~RobotPostures();
        
        void moveToinitialWalkingPosture(       IEncoders *iencs_left,
                                                IEncoders *iencs_right,
                                         IPositionControl *ipos_left,
                                         IPositionControl *ipos_right,
                                        yarp::sig::Vector JointsOffset,
                                        yarp::sig::Vector Des_ljoints,
                                        yarp::sig::Vector Des_rjoints);


        void moveTofinalWalkingPosture(         IEncoders *iencs_left,
                                                IEncoders *iencs_right,
                                         IPositionControl *ipos_left,
                                         IPositionControl *ipos_right,
                                        yarp::sig::Vector JointsOffset,
                                        yarp::sig::Vector Des_ljoints,
                                        yarp::sig::Vector Des_rjoints);


        void moveToHeightBasedPosture(      IEncoders *iencs_left,
                                            IEncoders *iencs_right,
                                     IPositionControl *ipos_left,
                                     IPositionControl *ipos_right,
                                               Vector JointsOffset,
                                               double CoMHeight);


        void moveToLegsPosture(       IEncoders *iencs_left,
                                      IEncoders *iencs_right,
                               IPositionControl *ipos_left,
                               IPositionControl *ipos_right,
                              yarp::sig::Vector JointsOffset,
                              yarp::sig::Vector Des_ljoints,
                              yarp::sig::Vector Des_rjoints);


};



// // ===============================================================================================

// // ===============================================================================================
class CommandsWriter
{

        double HipRollFactor;
        double HipYawFactor;
        double AnkleRollFactor;

          bool useCmdSmoother;
          bool useCmdOffset;
          bool SendLegsCmds;
          bool SendArmsCmds;
        string RobotName;
           int nbInterpolPts;
          bool useVelocityIK;
        double delta_theta_max;

    
    public:

        // legs
        // yarp::sig::Vector commands_left_leg;
        // yarp::sig::Vector commands_right_leg;

        yarp::sig::Vector previous_commands_left_leg;
        yarp::sig::Vector previous_commands_right_leg;

        yarp::sig::Vector Sent_commands_left_leg;
        yarp::sig::Vector Sent_commands_right_leg;

        // arms
            // position commands
        // yarp::sig::Vector commands_left_arm;
        // yarp::sig::Vector commands_right_arm;
        //     // torque commands
        // yarp::sig::Vector torqueCmd_left_arm;
        // yarp::sig::Vector torqueCmd_right_arm;

        yarp::sig::Vector previous_commands_left_arm;
        yarp::sig::Vector previous_commands_right_arm;

        yarp::sig::Vector Sent_commands_left_arm;
        yarp::sig::Vector Sent_commands_right_arm;

        // torso
        // yarp::sig::Vector commands_torso;


        // Cubic interpolator of the position joint commands to smooth them
        CubicInterpolator CmdSmoother;


    CommandsWriter();
    ~CommandsWriter();

    void InitCommendWriter(    InitBalWlkParameters *Parameters,
                           ControlledDevices  &botDevices); //,
                          // CubicInterpolator  CmdSmoother);

    void WriteLegsCommands(yarp::sig::Vector DesiredLeftLeg_joints, 
                           yarp::sig::Vector DesiredRightLeg_joints,
                           ControlledDevices &botDevices);   
                           //CubicInterpolator CmdSmoother,
                           
    
    void WriteArmsCommands( yarp::sig::Vector DesiredLeftArm_joints, 
                            yarp::sig::Vector DesiredRightArm_joints,
                            ControlledDevices &botDevices); 
                            //CubicInterpolator , 

    void WriteArmsVelocityCommands( Eigen::VectorXd des_jts_velocity_left_arm, 
                                    Eigen::VectorXd des_jts_velocity_right_arm,
                                    ControlledDevices &botDevices);
                            

    
 
};

#endif // CommunicationControl_H