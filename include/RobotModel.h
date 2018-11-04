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


// 9. RobotModel
// 	9.1. RobotKinematics
// 	9.2. SubLimbsUpperBody
// 	9.3. ArmsForceTorqueKinChain

#ifndef RobotModel_H
#define RobotModel_H


#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include "CommunicationControl.h"


using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace yarp::os;

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
// ==========================================================================

class RobotKinematics
{

	public:

	// Create kinematic chains for the robot 
    iCub::iKin::iKinChain *LeftLegChain;		// left leg
    iCub::iKin::iKinChain *RightLegChain;		// right leg 
    iCub::iKin::iKinChain *Left_Arm_Chain;		// left arm
    iCub::iKin::iKinChain *Right_Arm_Chain;		// right arm
    iCub::iKin::iKinChain *Torso_Chain;			// torso   

    // Create kinematics objects for the robot
	iCub::iKin::iCubLeg   *limb_L_Leg;			// iCubLeg: 	left leg
	iCub::iKin::iCubLeg   *limb_R_Leg;			// iCubLeg: 	right leg
	iCub::iKin::iCubArm   *limb_L_Arm;			// iCubArm: 	left arm
	iCub::iKin::iCubArm   *limb_R_Arm;			// iCubArm: 	right arm
	iCub::iKin::iCubTorso *limb_Torso;			// iCubTorso: 	torso

	// instantiate a IPOPT solver for arms and legs inverse kinematic
   	iCub::iKin::iKinIpOptMin *left_leg_solver;
   	iCub::iKin::iKinIpOptMin *right_leg_solver;

   	iCub::iKin::iKinIpOptMin *left_arm_solver;
   	iCub::iKin::iKinIpOptMin *right_arm_solver;


    // Vectors of desired legs joints
    yarp::sig::Vector DesiredLeftLeg_joints;
    yarp::sig::Vector DesiredRightLeg_joints;
    yarp::sig::Vector DesiredLeftArm_joints;
    yarp::sig::Vector DesiredRightArm_joints;

    yarp::sig::Vector previous_encoders_left_leg; 
	yarp::sig::Vector previous_encoders_right_leg; 
	Eigen::VectorXd jts_velocity_lleg; 
	Eigen::VectorXd jts_velocity_rleg;

	//
	firstOrderIntegrator *Filter_lleg_jtsVelo;
    firstOrderIntegrator *Filter_rleg_jtsVelo; 

	
	RobotKinematics(ControlledDevices &botDevices);
	~RobotKinematics();

	void Initialize(ControlledDevices &botDevices, double SamplingTime);
	void InitLeftLegInvKin();
	void InitRightLegInvKin();
	void InitLeftArmInvKin();
	void InitRightArmInvKin();
	
	yarp::sig::Vector ComputeLeftLegInvKin(	yarp::sig::Vector DesiredLeftLegPoseAsAxisAngles, 
											yarp::sig::Vector encoders_left_leg,	
														 bool useSensors);

	yarp::sig::Vector ComputeRightLegInvKin(yarp::sig::Vector DesiredRightLegPoseAsAxisAngles,
											yarp::sig::Vector encoders_right_leg,
									 					 bool useSensors);

	yarp::sig::Vector ComputeLeftArmInvKin(	yarp::sig::Vector DesiredLeftArmPoseAsAxisAngles,
											yarp::sig::Vector encoders_left_arm,
													   	 bool useSensors);

	yarp::sig::Vector ComputeRightArmInvKin(yarp::sig::Vector DesiredRightArmPoseAsAxisAngles,
											yarp::sig::Vector encoders_right_arm, 	
											 			 bool useSensors);

	void ComputeLegsInvKin(	yarp::sig::Vector DesiredLeftLegPoseAsAxisAngles,
							yarp::sig::Vector DesiredRightLegPoseAsAxisAngles, 
							yarp::sig::Vector encoders_left_leg, 
					 	   	yarp::sig::Vector encoders_right_leg,
										 bool useSensors);

	void ComputeArmsInvKin(	yarp::sig::Vector DesiredLeftArmPoseAsAxisAngles, 
							yarp::sig::Vector DesiredRightArmPoseAsAxisAngles, 
							yarp::sig::Vector encoders_left_arm, 
						   	yarp::sig::Vector encoders_right_arm,
										 bool useSensors);


	void UpdateLeftLegKinChain(	yarp::sig::Vector encoders_left_leg);

	void UpdateRightLegKinChain(yarp::sig::Vector encoders_right_leg);

	void UpdateLeftArmKinChain(	yarp::sig::Vector encoders_left_arm);

	void UpdateRightArmKinChain(yarp::sig::Vector encoders_right_arm);

	void UpdateTorsoKinChain(	yarp::sig::Vector encoders_torso);

	void UpdateAllRobotChains(	yarp::sig::Vector encoders_left_leg,
								yarp::sig::Vector encoders_right_leg,
								yarp::sig::Vector encoders_left_arm,
								yarp::sig::Vector encoders_right_arm,
								yarp::sig::Vector encoders_torso);

	void UpdateLegsJointsStates(yarp::sig::Vector encoders_left_leg,
								yarp::sig::Vector encoders_right_leg,
								double dt,
								bool useFilter);
	
};


// ====================================================================================================
// INVERSE KINEMACIS
// ====================================================================================================
class InverseKinematicsSolver
{
	public:


		// other inverse kinematics solver
		int count_max;
		double epsilon;
		double virtual_sampTime;
		double virtual_gain;

		VectorXd virtual_jts_velo;
		iCub::iKin::iKinChain *Chain;

		InverseKinematicsSolver();
		~InverseKinematicsSolver();

		void InitializeIK(iCub::iKin::iKinChain *Chain_, double gain_, int count_max_, double epilon_, double step_);
		VectorXd getPoseError_d_H_c(Matrix4d d_H_c);
		Matrix6d getInteractionMxForAxisAngle(MatrixXd d_H_c);
		yarp::sig::Vector get_IKsolution(//iCub::iKin::iKinChain *Chain,
										yarp::sig::Vector desPose, 
										yarp::sig::Vector encoders_chain_joints, 
										bool isDirectJacobian);

};

// =====================================================================================================
/**
 * upper limbs Kinematics
 */ 
// =====================================================================================================

// these classes inherit from iKinLimb and implement the 
// serial chain from the root link to the first elbow 
// joint

// class subLimb_LeftArm : public iKinLimb
// {
//     public:
//         subLimb_LeftArm() : iKinLimb()
//         {
//             allocate("just a string");
//         }

//     protected:
//         virtual void allocate(const string &_type)
//         {
//             // the type is used to discriminate between left and right limb

//             // you have to specify the rototranslational matrix H0 from the origin
//             // to the root reference so as from iCub specs.
//             yarp::sig::Matrix H0(4,4);
//             H0.zero();
//             H0(0,1)=-1.0;
//             H0(1,2)=-1.0;
//             H0(2,0)=1.0;
//             H0(3,3)=1.0;
//             setH0(H0);

//             // define the links in standard D-H convention
//             //                             A,        D,     alpha,           offset(*),          min theta,          max theta
//             pushLink(new iKinLink(     0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD)); 
//             pushLink(new iKinLink(       0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink(       0.0,  0.10774, -M_PI/2.0,            M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,           -M_PI/2.0,   0.0*CTRL_DEG2RAD, 160.8*CTRL_DEG2RAD));
//             pushLink(new iKinLink(     0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));

//             // pushLink(new iKinLink(    -0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
//             // pushLink(new iKinLink(       0.0,   0.1413,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
//             // pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -65.0*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));
//             // pushLink(new iKinLink(    0.0625, -0.02598,       0.0,                 0.0, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));

//             // (*) remind that offset is added to theta before computing the rototranslational matrix    
//             // usually the first three links which describes the torso kinematic come
//             // as blocked, i.e. they do not belong to the set of arm's dof.
//             blockLink(0,0.0);
//             blockLink(1,0.0);
//             blockLink(2,0.0);
//         }
// };


// class subLimb_RightArm : public iKinLimb
// {
//     public:
//         subLimb_RightArm() : iKinLimb()
//         {
//             allocate("just a string");
//         }

//     protected:
//         virtual void allocate(const string &_type)
//         {
//             // the type is used to discriminate between left and right limb

//             // you have to specify the rototranslational matrix H0 from the origin
//             // to the root reference so as from iCub specs.
//             yarp::sig::Matrix H0(4,4);
//             H0.zero();
//             H0(0,1)=-1.0;
//             H0(1,2)=-1.0;
//             H0(2,0)=1.0;
//             H0(3,3)=1.0;
//             setH0(H0);

//             // define the links in standard D-H convention
//             //                             A,        D,     alpha,           offset(*),          min theta,          max theta
//             pushLink(new iKinLink(     0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink(       0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink(       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
//             pushLink(new iKinLink(       0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,   0.0*CTRL_DEG2RAD, 160.8*CTRL_DEG2RAD));
//             pushLink(new iKinLink(    -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));

//             // pushLink(new iKinLink(     0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
//             // pushLink(new iKinLink(       0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));  // -- -90.0  90.0
//             // pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -65.0*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));  // -- -90.0  00.0
//             // pushLink(new iKinLink(    0.0625,  0.02598,       0.0,                M_PI, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));  // -- -20.0  40.0
            
//             // (*) remind that offset is added to theta before computing the rototranslational matrix    
//             // usually the first three links which describes the torso kinematic come
//             // as blocked, i.e. they do not belong to the set of arm's dof.
//             blockLink(0,0.0);
//             blockLink(1,0.0);
//             blockLink(2,0.0);
//         }
// };


// // =====================================================================================================
// // Kinematic chain from the robot root to the arm force/tirque sensor
// // =====================================================================================================

// class ArmsForceTorqueKinChain
// {

// 	public : 

// 	MatrixXd T_left_ArmFTsensor_root;
// 	MatrixXd T_right_ArmFTsensor_root;

// 	MatrixXd Rot_left_ArmFTsensor_root;
// 	MatrixXd Rot_right_ArmFTsensor_root;

// 	MatrixXd WrenchTrsf_left_ArmFTsensor_root;
// 	MatrixXd WrenchTrsf_right_ArmFTsensor_root;

// 	ArmsForceTorqueKinChain();

// 	~ArmsForceTorqueKinChain();

// 	void ComputeHomogeneousTransforms(	iCub::iKin::iKinChain *left_elbow_chain,
// 										iCub::iKin::iKinChain *right_elbow_chain);
	

// 	void ComputeWrenchTransforms( iCub::iKin::iKinChain *left_elbow_chain,
// 								  iCub::iKin::iKinChain *right_elbow_chain);
	

// };


#endif // RobotModel_H

