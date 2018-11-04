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

#include "RobotModel.h"

Matrix4d yarpPose2EigenHmatrix(yarp::sig::Vector yarpPoseVect)
{
	Matrix4d H_out; 	H_out.setIdentity(4,4);
	// position
	H_out.block<3,1>(0,3) << yarpPoseVect[0], yarpPoseVect[1], yarpPoseVect[2];
	// Orientation
	Eigen::Vector3d Axis;
	Axis << yarpPoseVect[3], yarpPoseVect[4], yarpPoseVect[5];
	H_out.block<3,3>(0,0) = Eigen::AngleAxisd(yarpPoseVect[6], Axis).toRotationMatrix();

	return H_out;
}

// ==========================================================================

RobotKinematics::RobotKinematics(ControlledDevices &botDevices): LeftLegChain(0)
															  , RightLegChain(0)
															  , Left_Arm_Chain(0)
															  , Right_Arm_Chain(0)
															  , Torso_Chain(0)
															  , limb_L_Leg(0)
															  , limb_R_Leg(0)
															  , limb_L_Arm(0)
															  , limb_R_Arm(0)
															  , limb_Torso(0)
															  , left_leg_solver(0)
															  , right_leg_solver(0)
															  , left_arm_solver(0)
															  , right_arm_solver(0)
															  , DesiredLeftLeg_joints(botDevices.lljoints)
															  , DesiredRightLeg_joints(botDevices.rljoints)
															  , DesiredLeftArm_joints(botDevices.lajoints)
															  , DesiredRightArm_joints(botDevices.rajoints){}


RobotKinematics::~RobotKinematics()
{
	if (left_leg_solver)
    {
        delete left_leg_solver;
        left_leg_solver = 0;
    }

    if (right_leg_solver)
    {
        delete right_leg_solver;
        right_leg_solver = 0;
    }

    if (left_arm_solver)
    {
        delete left_arm_solver;
        left_arm_solver = 0;
    }

    if (right_arm_solver)
    {
        delete right_arm_solver;
        right_arm_solver = 0;
    }

    if (Filter_lleg_jtsVelo)
    {
        delete Filter_lleg_jtsVelo;
        Filter_lleg_jtsVelo = 0;
    }

    if (Filter_rleg_jtsVelo)
    {
        delete Filter_rleg_jtsVelo;
        Filter_rleg_jtsVelo = 0;
    }

}

void RobotKinematics::Initialize(ControlledDevices &botDevices, double SamplingTime)
{
	// Create kinematic chains for the legs
    LeftLegChain    = new iCub::iKin::iKinLimb();	// left leg
    RightLegChain   = new iCub::iKin::iKinLimb();	// right leg
    Left_Arm_Chain  = new iCub::iKin::iKinLimb();	// left arm
    Right_Arm_Chain = new iCub::iKin::iKinLimb();	// right arm
    Torso_Chain		= new iCub::iKin::iKinLimb();	// torso

    // Create and initializing the kinematics objects for the robot
	limb_L_Leg = new iCub::iKin::iCubLeg("left_v2.5");	// iCubLeg: 	left leg
	limb_R_Leg = new iCub::iKin::iCubLeg("right_v2.5");	// iCubLeg: 	right leg
	limb_L_Arm = new iCub::iKin::iCubArm("left");		// iCubArm: 	left arm
	limb_R_Arm = new iCub::iKin::iCubArm("right");		// iCubArm: 	right arm
	limb_Torso = new iCub::iKin::iCubTorso("");			// iCubTorso: 	torso

	// 
	LeftLegChain  	= limb_L_Leg->asChain();		// left leg
	RightLegChain 	= limb_R_Leg->asChain();		// right leg
	Left_Arm_Chain  = limb_L_Arm->asChain();		// left arm
	Right_Arm_Chain = limb_R_Arm->asChain();		// right arm
	Torso_Chain     = limb_Torso->asChain();		// torso

	DesiredLeftLeg_joints.resize(LeftLegChain->getDOF());
    DesiredRightLeg_joints.resize(RightLegChain->getDOF());
    DesiredLeftArm_joints.resize(Left_Arm_Chain->getDOF());
    DesiredRightArm_joints.resize(Right_Arm_Chain->getDOF());

    // getting actual joints values
    botDevices.iencs_left_leg->getEncoders(botDevices.encoders_left_leg.data());
    botDevices.iencs_right_leg->getEncoders(botDevices.encoders_right_leg.data());
    botDevices.iencs_left_arm->getEncoders(botDevices.encoders_left_arm.data());
    botDevices.iencs_right_arm->getEncoders(botDevices.encoders_right_arm.data());

    //
    DesiredLeftLeg_joints  = LeftLegChain->getAng();
    DesiredRightLeg_joints = RightLegChain->getAng();
    DesiredLeftArm_joints  = Left_Arm_Chain->getAng();
    DesiredRightArm_joints = Right_Arm_Chain->getAng();

	// instantiate a IPOPT solver for inverse kinematic
   	// for both translational and rotational part
   	// left_leg_solver  = new iCub::iKin::iKinIpOptMin( *LeftLegChain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    // left_leg_solver->setUserScaling(true,100.0,100.0,100.0);
    // right_leg_solver = new iCub::iKin::iKinIpOptMin( *RightLegChain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    // right_leg_solver->setUserScaling(true,100.0,100.0,100.0);
    // 
    previous_encoders_left_leg.resize(botDevices.encoders_left_leg.size()); 
    previous_encoders_left_leg = botDevices.encoders_left_leg;
	previous_encoders_right_leg.resize(botDevices.encoders_right_leg.size()); 
	previous_encoders_right_leg = botDevices.encoders_right_leg;
	//
	jts_velocity_lleg.resize(botDevices.encoders_left_leg.size()); 
	jts_velocity_lleg.setZero();
	jts_velocity_rleg.resize(botDevices.encoders_right_leg.size()); 
	jts_velocity_rleg.setZero();

	//
	Filter_lleg_jtsVelo = new firstOrderIntegrator(SamplingTime, 3., 3., jts_velocity_lleg);
    Filter_rleg_jtsVelo = new firstOrderIntegrator(SamplingTime, 3., 3., jts_velocity_rleg);

}

// Initialize the inverse kinematic solver for the left leg
void RobotKinematics::InitLeftLegInvKin()
{
	// instantiate a IPOPT solver for inverse kinematic
   	// for both translational and rotational part
   	left_leg_solver  = new iCub::iKin::iKinIpOptMin( *LeftLegChain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    left_leg_solver->setUserScaling(true,100.0,100.0,100.0);
    
}

// Initialize the inverse kinematic solver for the right leg
void RobotKinematics::InitRightLegInvKin()
{
	// instantiate a IPOPT solver for inverse kinematic
   	// for both translational and rotational part
    right_leg_solver = new iCub::iKin::iKinIpOptMin( *RightLegChain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    right_leg_solver->setUserScaling(true,100.0,100.0,100.0);
}

// Initialize the inverse kinematic solver for the left arm
void RobotKinematics::InitLeftArmInvKin()
{
	// instantiate a IPOPT solver for inverse kinematic
   	// for both translational and rotational part
   	left_arm_solver  = new iCub::iKin::iKinIpOptMin( *Left_Arm_Chain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    left_arm_solver->setUserScaling(true,100.0,100.0,100.0);
    
}

// Initialize the inverse kinematic solver for the right arm
void RobotKinematics::InitRightArmInvKin()
{
	// instantiate a IPOPT solver for inverse kinematic
   	// for both translational and rotational part
    right_arm_solver = new iCub::iKin::iKinIpOptMin( *Right_Arm_Chain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    right_arm_solver->setUserScaling(true,100.0,100.0,100.0);
}




// Compute the inverse kinematics of the left leg
yarp::sig::Vector RobotKinematics::ComputeLeftLegInvKin(yarp::sig::Vector DesiredLeftLegPoseAsAxisAngles, 
														yarp::sig::Vector encoders_left_leg,
																	 bool useSensors)
{  	
	if (useSensors)
	{
		DesiredLeftLeg_joints  = left_leg_solver->solve(LeftLegChain->getAng(), DesiredLeftLegPoseAsAxisAngles); 
	}
	else
	{
		DesiredLeftLeg_joints  = left_leg_solver->solve(DesiredLeftLeg_joints, DesiredLeftLegPoseAsAxisAngles); 
	}
     
    return DesiredLeftLeg_joints;
}

// Compute the inverse kinematics of the right leg
yarp::sig::Vector RobotKinematics::ComputeRightLegInvKin(yarp::sig::Vector DesiredRightLegPoseAsAxisAngles,
														 yarp::sig::Vector encoders_right_leg,
																	  bool useSensors)
{   
    if (useSensors)
    {
    	DesiredRightLeg_joints  = right_leg_solver->solve(RightLegChain->getAng(), DesiredRightLegPoseAsAxisAngles);
    }
    else
    {
    	DesiredRightLeg_joints  = right_leg_solver->solve(DesiredRightLeg_joints, DesiredRightLegPoseAsAxisAngles);
    }

    return DesiredRightLeg_joints;
}

// Compute the inverse kinematics of the two legs
void RobotKinematics::ComputeLegsInvKin(yarp::sig::Vector DesiredLeftLegPoseAsAxisAngles,
										yarp::sig::Vector DesiredRightLegPoseAsAxisAngles, 
										yarp::sig::Vector encoders_left_leg, 
								 	   	yarp::sig::Vector encoders_right_leg,
													 bool useSensors)
{
	RobotKinematics::ComputeLeftLegInvKin( DesiredLeftLegPoseAsAxisAngles,  encoders_left_leg,  useSensors);
	RobotKinematics::ComputeRightLegInvKin(DesiredRightLegPoseAsAxisAngles, encoders_right_leg, useSensors);
}




// Compute the inverse kinematics of the left arm
yarp::sig::Vector RobotKinematics::ComputeLeftArmInvKin(yarp::sig::Vector DesiredLeftArmPoseAsAxisAngles,
														yarp::sig::Vector encoders_left_arm,
																	 bool useSensors)
{
	if (useSensors)
    {
    	DesiredLeftArm_joints  = left_arm_solver->solve(Left_Arm_Chain->getAng(), DesiredLeftArmPoseAsAxisAngles);
    }
    else
    {
    	DesiredLeftArm_joints  = left_arm_solver->solve(DesiredLeftArm_joints, DesiredLeftArmPoseAsAxisAngles);
    }
  
    return DesiredLeftArm_joints;
}

// Compute the inverse kinematics of the right arm
yarp::sig::Vector RobotKinematics::ComputeRightArmInvKin(yarp::sig::Vector DesiredRightArmPoseAsAxisAngles,
														 yarp::sig::Vector encoders_right_arm,
																	  bool useSensors)
{
	if (useSensors)
    {
    	DesiredRightArm_joints  = right_arm_solver->solve(Right_Arm_Chain->getAng(), DesiredRightArmPoseAsAxisAngles);
    }
    else
    {
    	DesiredRightArm_joints  = right_arm_solver->solve(DesiredRightArm_joints, DesiredRightArmPoseAsAxisAngles);
    }

    return DesiredRightArm_joints;
}

// Compute the inverse kinematics of the two arms
void RobotKinematics::ComputeArmsInvKin(yarp::sig::Vector DesiredLeftArmPoseAsAxisAngles, 
										yarp::sig::Vector DesiredRightArmPoseAsAxisAngles, 
										yarp::sig::Vector encoders_left_arm, 
									   	yarp::sig::Vector encoders_right_arm,
													 bool useSensors)
{
	RobotKinematics::ComputeLeftArmInvKin( DesiredLeftArmPoseAsAxisAngles,  encoders_left_arm,  useSensors);
	RobotKinematics::ComputeRightArmInvKin(DesiredRightArmPoseAsAxisAngles, encoders_right_arm, useSensors);
}


// Update the kinematic chain with the measured values of the left leg joints
void RobotKinematics::UpdateLeftLegKinChain(yarp::sig::Vector encoders_left_leg)
{
	// update the legs chains
	LeftLegChain->setAng(CTRL_DEG2RAD * encoders_left_leg); 
}

// Update the kinematic chain with the measured values of the right leg joints
void RobotKinematics::UpdateRightLegKinChain(yarp::sig::Vector encoders_right_leg)
{
	// update the legs chains
	RightLegChain->setAng(CTRL_DEG2RAD * encoders_right_leg);
}

// Update the kinematic chain with the measured values of the left arm joints
void RobotKinematics::UpdateLeftArmKinChain(yarp::sig::Vector encoders_left_arm)
{
	// update the arms chains
	Left_Arm_Chain->setAng(CTRL_DEG2RAD  * encoders_left_arm);
}

// Update the kinematic chain with the measured values of the right arm joints
void RobotKinematics::UpdateRightArmKinChain(yarp::sig::Vector encoders_right_arm)
{
	// update the arms chains
	Right_Arm_Chain->setAng(CTRL_DEG2RAD * encoders_right_arm);
}

// Update the kinematic chain with the measured values of the torso joints
void RobotKinematics::UpdateTorsoKinChain(yarp::sig::Vector encoders_torso)
{
	// update the torso chain
	Torso_Chain->setAng(CTRL_DEG2RAD * encoders_torso);
}

// Update the kinematic chain with the measured values of all the robot joints
void RobotKinematics::UpdateAllRobotChains(	yarp::sig::Vector encoders_left_leg,
											yarp::sig::Vector encoders_right_leg,
											yarp::sig::Vector encoders_left_arm,
											yarp::sig::Vector encoders_right_arm,
											yarp::sig::Vector encoders_torso)
{
	// update all the robot chains
	RobotKinematics::UpdateLeftLegKinChain(encoders_left_leg);
	RobotKinematics::UpdateRightLegKinChain(encoders_right_leg);
	RobotKinematics::UpdateLeftArmKinChain(encoders_left_arm);
	RobotKinematics::UpdateRightArmKinChain(encoders_right_arm);
	RobotKinematics::UpdateTorsoKinChain(encoders_torso);
	
}

// Update the joints state (position and velocity) with the measured values of the legs joints
void RobotKinematics::UpdateLegsJointsStates(yarp::sig::Vector encoders_left_leg,
											yarp::sig::Vector encoders_right_leg,
											double dt,
											bool useFilter)
{
	// update the legs chains
	LeftLegChain->setAng(CTRL_DEG2RAD * encoders_left_leg); 
	RightLegChain->setAng(CTRL_DEG2RAD * encoders_right_leg);
	//
	// Approximate the joints velocity from joints positions (numeracal derivative)
	yarp::sig::Vector qDot_lleg_hat = CTRL_DEG2RAD * (1./dt)*(encoders_left_leg -  previous_encoders_left_leg);
	yarp::sig::Vector qDot_rleg_hat = CTRL_DEG2RAD * (1./dt)*(encoders_right_leg - previous_encoders_right_leg);
	//
	previous_encoders_left_leg  = encoders_left_leg;
	previous_encoders_right_leg = encoders_right_leg;

	// get the joint velocity
	for (int i=0; i<RightLegChain->getDOF(); i++)
    {
        jts_velocity_lleg(i) = qDot_lleg_hat[i];
        jts_velocity_rleg(i) = qDot_rleg_hat[i];
    }

    if(useFilter)
    {
    	jts_velocity_lleg = Filter_lleg_jtsVelo->getRK4Integral(jts_velocity_lleg);
    	jts_velocity_rleg = Filter_rleg_jtsVelo->getRK4Integral(jts_velocity_rleg);
    }


}

// ====================================================================================================
// INVERSE KINEMACIS
// ====================================================================================================

InverseKinematicsSolver::InverseKinematicsSolver(){}

InverseKinematicsSolver::~InverseKinematicsSolver(){}

void InverseKinematicsSolver::InitializeIK(iCub::iKin::iKinChain *Chain_, double gain_, int count_max_, double epilon_, double step_)
{
	//
	virtual_sampTime = step_;
	// epsilon = 1e-4;
	// count_max  = 10;
	// virtual_gain = 0.3;

	epsilon 	 = epilon_;
	count_max  	 = count_max_;
	virtual_gain = gain_;
	//
	Chain = Chain_;
	//
	virtual_jts_velo.resize(Chain->getDOF());
	//

}

// Vector of feature error
VectorXd InverseKinematicsSolver::getPoseError_d_H_c(Matrix4d d_H_c)
{
    // extracrion of the rotation
    Eigen::AngleAxisd d_AxisAngle_c(d_H_c.block<3,3>(0,0));

    Eigen::VectorXd d_eta_c(6);
    d_eta_c.segment(0,3) << d_H_c(0,3), d_H_c(1,3), d_H_c(2,3);

    Eigen::Vector3d d_Axis_c = d_AxisAngle_c.axis();
    d_eta_c(3) = d_Axis_c(0) * d_AxisAngle_c.angle();
    d_eta_c(4) = d_Axis_c(1) * d_AxisAngle_c.angle();
    d_eta_c(5) = d_Axis_c(2) * d_AxisAngle_c.angle();

            
    return d_eta_c;

}
//
Matrix6d InverseKinematicsSolver::getInteractionMxForAxisAngle(MatrixXd d_H_c)
{
    /* This function computes the Jacobian associated with a rigid transformation
     * represented as homogeneous transformation matrix from the current frame to
     * the desired frame (d_H_c) and where the orientation is represented with an Axis/Angle
     */

    // Jacobian associated with the configuration error
    MatrixXd Jac_Mu_Theta;

    Jac_Mu_Theta.resize(6,6);
    Jac_Mu_Theta.setZero(6,6);

    // extracrion of the rotation
    Eigen::AngleAxisd d_AxisAngle_c(d_H_c.block<3,3>(0,0));

    // function sinc(theta) and sinc(theta/2)
    double sinc_theta, sinc_theta_2;

    sinc_theta   = sin(d_AxisAngle_c.angle() + 1e-6)/(d_AxisAngle_c.angle() + 1e-6);
    sinc_theta_2 = sin((d_AxisAngle_c.angle() + 1e-6)/2.)/((d_AxisAngle_c.angle() + 1e-6)/2.);
        
    Matrix3d Skew_Mu;
    Skew_Mu.resize(3,3);
    Skew_Mu.setZero(3,3);

    Eigen::Vector3d d_Axis_c = d_AxisAngle_c.axis();
    Skew_Mu(0,1) = -d_Axis_c(2);
    Skew_Mu(0,2) =  d_Axis_c(1);
    Skew_Mu(1,0) =  d_Axis_c(2);
    Skew_Mu(1,2) = -d_Axis_c(0);
    Skew_Mu(2,0) = -d_Axis_c(1);
    Skew_Mu(2,1) =  d_Axis_c(0);

    // Jacobian of the rotation
    Matrix3d L_Mu_Theta;
    L_Mu_Theta.resize(3,3);
    L_Mu_Theta.setIdentity(3,3);

    L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c.angle()/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

    // Building the overall jacobian
    Matrix6d InteractionMx_Mu_Theta; InteractionMx_Mu_Theta.setZero();
    InteractionMx_Mu_Theta.block<3,3>(0,0) = d_H_c.block<3,3>(0,0);
    InteractionMx_Mu_Theta.block<3,3>(3,3) = L_Mu_Theta;

    return InteractionMx_Mu_Theta;

}

yarp::sig::Vector InverseKinematicsSolver::get_IKsolution(yarp::sig::Vector desPose, yarp::sig::Vector encoders_chain_joints, bool isDirectJacobian)
{
	
	// create a virtual joints vector
	yarp::sig::Vector virt_chain_joints(encoders_chain_joints.size());
	virt_chain_joints = CTRL_DEG2RAD *encoders_chain_joints;
	// compute the homogeneous matrix of the desired eef pose wrt the base frame
	Matrix4d b_H_d = yarpPose2EigenHmatrix(desPose);
	//  homogeneous transformation related to the  current pose of the eef wrt the base
	Matrix4d b_H_c;
	// relative pose between the current and desired end effector
	Matrix4d d_H_c;
	//
	Matrix6d eef_Rot6D_b;
			 eef_Rot6D_b.setIdentity();
	yarp::sig::Vector curPose(7);
	//
	int count = 0;

	double error_norm = 1.0;
		
	// compute the pseudo inverse matrix
    MatrixPseudoInverse2 MxPsdInv;

    //
    Vector6d gain_vector;   gain_vector.setOnes();
   gain_vector.head(3) *= -2.0*virtual_gain;
   gain_vector.tail(3) *= -virtual_gain;
	//
	while((count <= count_max) && (error_norm > epsilon))
	{
		
		// update the chain with the virtual joints positions
		Chain->setAng(virt_chain_joints);

		// get the current end effector pose in the base frame
		curPose = Chain->EndEffPose();
		// get the homogeneous transformation related to the  current pose of the eef wrt the base
		b_H_c = yarpPose2EigenHmatrix(curPose);
		// get relative pose between the current and desired end effector
		d_H_c = b_H_d.inverse() * b_H_c;

		Matrix4d c_H_d = d_H_c.inverse();
        Eigen::AngleAxisd orient_error(d_H_c.block<3,3>(0,0));

		// compute the task error
		Vector6d error_pose = getPoseError_d_H_c(d_H_c); // in desired : in current

		// compute the norm of the error  // here psition and orientation (can be separared into two different norms)
		error_norm = error_pose.norm();

		// compute interaction matrix
		Matrix6d L_eta_chain = getInteractionMxForAxisAngle(d_H_c);

		// 6D rotation matrix from base to end effector
		eef_Rot6D_b.block<3,3>(0,0) = b_H_c.block<3,3>(0,0).transpose();
		eef_Rot6D_b.block<3,3>(3,3) = b_H_c.block<3,3>(0,0).transpose();

		// compute the task jacobian 
		// compute the chain jacobian
		yarp::sig::Matrix yChain_Jacobian =  Chain->GeoJacobian();

		// compute the joint veleocity
		Eigen::MatrixXd b_Jacobian_Mx_eef(yChain_Jacobian.rows(), yChain_Jacobian.cols()); 
		//
		for (int row=0; row < yChain_Jacobian.rows(); row++) 
		{
		    for(int col=0; col < yChain_Jacobian.cols(); col++) 
		    {
		        b_Jacobian_Mx_eef(row, col) = yChain_Jacobian(row, col);
		    }
		}

		// compute the joint veleocity
		Eigen::MatrixXd Pseudo_Jacobian_task(b_Jacobian_Mx_eef.cols(),b_Jacobian_Mx_eef.rows());
		Pseudo_Jacobian_task.setZero();

		// 
		if(isDirectJacobian)
		{
			// compute the joint veleocity : direct
			Eigen::MatrixXd Jacobian_task = L_eta_chain * eef_Rot6D_b * b_Jacobian_Mx_eef;
	        error_pose = gain_vector.asDiagonal() *error_pose;
			//error_pose *= -virtual_gain;
			virtual_jts_velo = Jacobian_task.colPivHouseholderQr().solve(error_pose);    				// QR

		}
		else
		{
			// compute the joint veleocity : indirect
			Matrix6d 	inv_L_eta_chain;
						inv_L_eta_chain.setZero();
						inv_L_eta_chain.block<3,3>(0,0) = d_H_c.block<3,3>(0,0).transpose();
			    		inv_L_eta_chain.block<3,3>(3,3) = MatrixXd::Identity(3,3);

			Vector6d virtual_eef_velo = -virtual_gain * eef_Rot6D_b.transpose() * inv_L_eta_chain * error_pose;
			// Vector6d virtual_eef_velo = -virtual_gain * error_pose;
			// compute the pseudo inverse of the jacobian
			MxPsdInv.get_HhQRPseudoInverse(b_Jacobian_Mx_eef, Pseudo_Jacobian_task);

			// compute the virtual joints velocity
			virtual_jts_velo = Pseudo_Jacobian_task * virtual_eef_velo;
			// virtual_jts_velo = b_Jacobian_Mx_eef.colPivHouseholderQr().solve(virtual_eef_velo);   					// QR
		}

		// compute the joint position variation
		Eigen::VectorXd delta_virtual_jts(virtual_jts_velo.rows());
		delta_virtual_jts = virtual_jts_velo * virtual_sampTime;

		// update the virtual joints positions
		for(int i=0; i<virt_chain_joints.size(); i++)
		{
			virt_chain_joints[i] += delta_virtual_jts(i);

			// set joint limits hard limits
			if(virt_chain_joints[i] > (*Chain)(i).getMax()){  // upper limit

				virt_chain_joints[i] = (*Chain)(i).getMax() - 0.01;

			} else if(virt_chain_joints[i] < (*Chain)(i).getMin()){ // lower limit

				virt_chain_joints[i] = (*Chain)(i).getMin() + 0.01;

			} 

		}
		count++;
		//
		
	}

	// set the joints of the chain back to the encoders values
	Chain->setAng(CTRL_DEG2RAD *encoders_chain_joints);

	return virt_chain_joints;
}

// ====================================================================================




// ====================================================================================


// ArmsForceTorqueKinChain::ArmsForceTorqueKinChain()
// {
	
// 	T_left_ArmFTsensor_root.resize(4,4);
// 	T_right_ArmFTsensor_root.resize(4,4);

// 	Rot_left_ArmFTsensor_root.resize(3,3);
// 	Rot_right_ArmFTsensor_root.resize(3,3);

// 	WrenchTrsf_left_ArmFTsensor_root.resize(6,6);
// 	WrenchTrsf_right_ArmFTsensor_root.resize(6,6);

// 	WrenchTrsf_left_ArmFTsensor_root.setIdentity(6,6);
// 	WrenchTrsf_right_ArmFTsensor_root.setIdentity(6,6);
// }

// ArmsForceTorqueKinChain::~ArmsForceTorqueKinChain() 
// {
// 	// if (left_elbow_chain)
// 	// {
// 	// 	delete left_elbow_chain;
// 	// 	left_elbow_chain = 0;
// 	// }

// 	// if (right_elbow_chain)
// 	// {
// 	// 	delete right_elbow_chain;
// 	// 	right_elbow_chain = 0;
// 	// }
// }

// void ArmsForceTorqueKinChain::ComputeHomogeneousTransforms(	iCub::iKin::iKinChain *left_elbow_chain,
// 															iCub::iKin::iKinChain *right_elbow_chain)
// {
// 	// // instantiating sub limbs objects 
// 	// Declaring Pose vectors of the two sub-limbs
// 	yarp::sig::Vector left_elbow_root_Pose;
// 	yarp::sig::Vector right_elbow_root_Pose;

// 	// // updating the chain
// 	// left_elbow_chain->setAng(CTRL_DEG2RAD * left_subchain_jts);
// 	// right_elbow_chain->setAng(CTRL_DEG2RAD * right_subchain_jts);

// 	// computing the Pose
// 	left_elbow_root_Pose  = left_elbow_chain->EndEffPose();
// 	right_elbow_root_Pose = right_elbow_chain->EndEffPose();

// 	yarp::sig::Matrix Rot_left_elbow_root;
// 	yarp::sig::Matrix Rot_right_elbow_root;

// 	Rot_left_elbow_root  = yarp::math::axis2dcm(left_elbow_root_Pose.subVector(3,6));
// 	Rot_right_elbow_root = yarp::math::axis2dcm(right_elbow_root_Pose.subVector(3,6));

// 	// Homogeneous transformations
// 	    // First elbow frame wrt. root frame
// 	Eigen::MatrixXd T_left_elbow_root(4,4),
// 	                T_right_elbow_root(4,4);

// 	T_left_elbow_root.setZero(4,4);      
// 	T_left_elbow_root(0,3) = left_elbow_root_Pose[0];
// 	T_left_elbow_root(1,3) = left_elbow_root_Pose[1];
// 	T_left_elbow_root(2,3) = left_elbow_root_Pose[2];
// 	T_left_elbow_root(3,3) = 1.;

// 	T_right_elbow_root.setZero(4,4);     
// 	T_right_elbow_root(0,3) = right_elbow_root_Pose[0];
// 	T_right_elbow_root(1,3) = right_elbow_root_Pose[1];
// 	T_right_elbow_root(2,3) = right_elbow_root_Pose[2];
// 	T_right_elbow_root(3,3) = 1.;

// 	// extraction of data from yarp matrix to Eigien matrix
// 	for (int row=0; row<3; row++)
// 	{
// 	    for (int col=0; col<3; col++)
// 	    {
// 	        T_left_elbow_root(row,col)  = Rot_left_elbow_root(row,col);
// 	        T_right_elbow_root(row,col) = Rot_right_elbow_root(row,col);
// 	    }
// 	}

// 	    // Force/Torque sensor frame wrt. first elbow frame  
// 	Eigen::MatrixXd T_left_FTsensor_elbow(4,4),
// 	                T_right_FTsensor_elbow(4,4);

// 	T_left_FTsensor_elbow.setZero(4,4);         T_left_FTsensor_elbow(3,3) = 1.;
// 	    // translation     
// 	T_left_FTsensor_elbow(0,3) = 0.015/2.;
// 	T_left_FTsensor_elbow(1,3) = 0.15228/2.;
// 	T_left_FTsensor_elbow(2,3) = 0.0;
// 	    // rotation
// 	T_left_FTsensor_elbow(0,0) =  1.0;
// 	T_left_FTsensor_elbow(1,2) = -1.0;
// 	T_left_FTsensor_elbow(2,1) =  1.0;


// 	T_right_FTsensor_elbow.setZero(4,4);        T_right_FTsensor_elbow(3,3) = 1.;    
// 	    // translation     
// 	T_right_FTsensor_elbow(0,3) = -0.015/2.;
// 	T_right_FTsensor_elbow(1,3) = -0.15228/2.;
// 	T_right_FTsensor_elbow(2,3) =  0.0;
// 	    // rotation
// 	T_right_FTsensor_elbow(0,0) = -1.0;
// 	T_right_FTsensor_elbow(1,2) =  1.0;
// 	T_right_FTsensor_elbow(2,1) =  1.0;

// 	// Homogeneous Transformations of the arms FT sensors frames wrt. the root frame
// 	// Eigen::MatrixXd T_left_ArmFTsensor_root(4,4),
// 	//                 T_right_ArmFTsensor_root(4,4);

// 	// compute the homogeneous transformations
// 	T_left_ArmFTsensor_root  = T_left_elbow_root  * T_left_FTsensor_elbow;
// 	T_right_ArmFTsensor_root = T_right_elbow_root * T_right_FTsensor_elbow;

// 	// extracting the rotation matrices
// 	Rot_left_ArmFTsensor_root  = T_left_ArmFTsensor_root.block(0,0, 3,3);
// 	Rot_right_ArmFTsensor_root = T_right_ArmFTsensor_root.block(0,0, 3,3);


// }

// void ArmsForceTorqueKinChain::ComputeWrenchTransforms( iCub::iKin::iKinChain *left_elbow_chain,
// 							  						   iCub::iKin::iKinChain *right_elbow_chain)
// {
	
// 	// computing first the homogeneous transformations
// 	ComputeHomogeneousTransforms( left_elbow_chain,
// 								  right_elbow_chain);

// 	// Force/Torque transformation matrices
//     // skew symmetric transformation matrices
// 	Eigen::MatrixXd SkewMx_left_ArmFTsensor_root(3,3),
// 	                SkewMx_right_ArmFTsensor_root(3,3);

// 	SkewMx_left_ArmFTsensor_root.setZero(3,3);
// 	SkewMx_left_ArmFTsensor_root(0,1) = -T_left_ArmFTsensor_root(2,3);
// 	SkewMx_left_ArmFTsensor_root(0,2) =  T_left_ArmFTsensor_root(1,3);
// 	SkewMx_left_ArmFTsensor_root(1,0) =  T_left_ArmFTsensor_root(2,3);
// 	SkewMx_left_ArmFTsensor_root(1,2) = -T_left_ArmFTsensor_root(0,3);
// 	SkewMx_left_ArmFTsensor_root(2,0) = -T_left_ArmFTsensor_root(1,3);
// 	SkewMx_left_ArmFTsensor_root(2,1) =  T_left_ArmFTsensor_root(0,3);

// 	SkewMx_right_ArmFTsensor_root.setZero(3,3);
// 	SkewMx_right_ArmFTsensor_root(0,1) = -T_right_ArmFTsensor_root(2,3);
// 	SkewMx_right_ArmFTsensor_root(0,2) =  T_right_ArmFTsensor_root(1,3);
// 	SkewMx_right_ArmFTsensor_root(1,0) =  T_right_ArmFTsensor_root(2,3);
// 	SkewMx_right_ArmFTsensor_root(1,2) = -T_right_ArmFTsensor_root(0,3);
// 	SkewMx_right_ArmFTsensor_root(2,0) = -T_right_ArmFTsensor_root(1,3);
// 	SkewMx_right_ArmFTsensor_root(2,1) =  T_right_ArmFTsensor_root(0,3);

// 	cout << " T_right_ArmFTsensor_root is : \n"<< T_right_ArmFTsensor_root << endl;

// 	// wrench transformation matrices

// 	    // left arm
// 	WrenchTrsf_left_ArmFTsensor_root.block(0,0, 3,3) = T_left_ArmFTsensor_root.block(0,0, 3,3);
// 	WrenchTrsf_left_ArmFTsensor_root.block(3,0, 3,3) = SkewMx_left_ArmFTsensor_root * T_left_ArmFTsensor_root.block(0,0, 3,3);
// 	WrenchTrsf_left_ArmFTsensor_root.block(3,3, 3,3) = T_left_ArmFTsensor_root.block(0,0, 3,3);
	
// 	    // right arm
// 	WrenchTrsf_right_ArmFTsensor_root.block(0,0, 3,3) = T_right_ArmFTsensor_root.block(0,0, 3,3);
// 	WrenchTrsf_right_ArmFTsensor_root.block(3,0, 3,3) = SkewMx_right_ArmFTsensor_root * T_right_ArmFTsensor_root.block(0,0, 3,3);
// 	WrenchTrsf_right_ArmFTsensor_root.block(3,3, 3,3) = T_right_ArmFTsensor_root.block(0,0, 3,3);
	

// }

