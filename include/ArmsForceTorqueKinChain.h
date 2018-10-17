

#ifndef ArmsForceTorqueKinChain_H
#define ArmsForceTorqueKinChain_H

#include <string>
#include <iostream>
#include <cmath>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "SubLimbsUpperBody.h"


using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace std;
using namespace Eigen;


class ArmsForceTorqueKinChain
{

	public : 

	MatrixXd T_left_ArmFTsensor_root;
	MatrixXd T_right_ArmFTsensor_root;

	MatrixXd Rot_left_ArmFTsensor_root;
	MatrixXd Rot_right_ArmFTsensor_root;

	MatrixXd WrenchTrsf_left_ArmFTsensor_root;
	MatrixXd WrenchTrsf_right_ArmFTsensor_root;

	ArmsForceTorqueKinChain()
	{
		
		T_left_ArmFTsensor_root.resize(4,4);
		T_right_ArmFTsensor_root.resize(4,4);

		Rot_left_ArmFTsensor_root.resize(3,3);
		Rot_right_ArmFTsensor_root.resize(3,3);

		WrenchTrsf_left_ArmFTsensor_root.resize(6,6);
		WrenchTrsf_right_ArmFTsensor_root.resize(6,6);

		WrenchTrsf_left_ArmFTsensor_root.setIdentity(6,6);
		WrenchTrsf_right_ArmFTsensor_root.setIdentity(6,6);
	}

	~ArmsForceTorqueKinChain() 
	{
		// if (left_elbow_chain)
		// {
		// 	delete left_elbow_chain;
		// 	left_elbow_chain = 0;
		// }

		// if (right_elbow_chain)
		// {
		// 	delete right_elbow_chain;
		// 	right_elbow_chain = 0;
		// }
	}

	void ComputeHomogeneousTransforms(	iCub::iKin::iKinChain *left_elbow_chain,
										iCub::iKin::iKinChain *right_elbow_chain)
	{
		// // instantiating sub limbs objects 
		// subLimb_LeftArm  root_left_elbow;
		// subLimb_RightArm root_right_elbow;

		// // Declaring kinematic chain objects
		// iCub::iKin::iKinChain *left_elbow_chain;
		// iCub::iKin::iKinChain *right_elbow_chain;

		// left_elbow_chain  = root_left_elbow.asChain();
		// right_elbow_chain = root_right_elbow.asChain();

		// int nL_dof = left_elbow_chain->getDOF();
		// int nR_dof = right_elbow_chain->getDOF();

		// Declaring Pose vectors of the two sub-limbs
		yarp::sig::Vector left_elbow_root_Pose;
		yarp::sig::Vector right_elbow_root_Pose;

		// // updating the chain
		// left_elbow_chain->setAng(CTRL_DEG2RAD * left_subchain_jts);
		// right_elbow_chain->setAng(CTRL_DEG2RAD * right_subchain_jts);

		// computing the Pose
		left_elbow_root_Pose  = left_elbow_chain->EndEffPose();
		right_elbow_root_Pose = right_elbow_chain->EndEffPose();

		yarp::sig::Matrix Rot_left_elbow_root;
		yarp::sig::Matrix Rot_right_elbow_root;

		Rot_left_elbow_root  = yarp::math::axis2dcm(left_elbow_root_Pose.subVector(3,6));
		Rot_right_elbow_root = yarp::math::axis2dcm(right_elbow_root_Pose.subVector(3,6));

		// Homogeneous transformations
		    // First elbow frame wrt. root frame
		Eigen::MatrixXd T_left_elbow_root(4,4),
		                T_right_elbow_root(4,4);

		T_left_elbow_root.setZero(4,4);      
		T_left_elbow_root(0,3) = left_elbow_root_Pose[0];
		T_left_elbow_root(1,3) = left_elbow_root_Pose[1];
		T_left_elbow_root(2,3) = left_elbow_root_Pose[2];
		T_left_elbow_root(3,3) = 1.;

		T_right_elbow_root.setZero(4,4);     
		T_right_elbow_root(0,3) = right_elbow_root_Pose[0];
		T_right_elbow_root(1,3) = right_elbow_root_Pose[1];
		T_right_elbow_root(2,3) = right_elbow_root_Pose[2];
		T_right_elbow_root(3,3) = 1.;

		// extraction of data from yarp matrix to Eigien matrix
		for (int row=0; row<3; row++)
		{
		    for (int col=0; col<3; col++)
		    {
		        T_left_elbow_root(row,col)  = Rot_left_elbow_root(row,col);
		        T_right_elbow_root(row,col) = Rot_right_elbow_root(row,col);
		    }
		}

		// Force/Torque sensor frame wrt. first elbow frame  
		Eigen::MatrixXd T_left_FTsensor_elbow(4,4),
		                T_right_FTsensor_elbow(4,4);

		// T_left_FTsensor_elbow.setZero(4,4);         T_left_FTsensor_elbow(3,3) = 1.;
		//     // translation     
		// T_left_FTsensor_elbow(0,3) = 0.015/2.;
		// T_left_FTsensor_elbow(1,3) = 0.15228/2.;
		// T_left_FTsensor_elbow(2,3) = 0.0;
		//     // rotation
		// T_left_FTsensor_elbow(0,0) =  1.0;
		// T_left_FTsensor_elbow(1,2) = -1.0;
		// T_left_FTsensor_elbow(2,1) =  1.0;


		// T_right_FTsensor_elbow.setZero(4,4);        T_right_FTsensor_elbow(3,3) = 1.;    
		//     // translation     
		// T_right_FTsensor_elbow(0,3) = -0.015/2.;
		// T_right_FTsensor_elbow(1,3) = -0.15228/2.;
		// T_right_FTsensor_elbow(2,3) =  0.0;
		//     // rotation
		// T_right_FTsensor_elbow(0,0) = -1.0;
		// T_right_FTsensor_elbow(1,2) =  1.0;
		// T_right_FTsensor_elbow(2,1) =  1.0;

		T_left_FTsensor_elbow.setZero(4,4);         T_left_FTsensor_elbow(3,3) = 1.;
		    // translation     
		T_left_FTsensor_elbow(0,3) = 0.00;
		T_left_FTsensor_elbow(1,3) = 0.15228-0.068;
		T_left_FTsensor_elbow(2,3) = 0.0;
		    // rotation
		T_left_FTsensor_elbow(0,0) =  1.0;
		T_left_FTsensor_elbow(1,2) = -1.0;
		T_left_FTsensor_elbow(2,1) =  1.0;


		T_right_FTsensor_elbow.setZero(4,4);        T_right_FTsensor_elbow(3,3) = 1.;    
		    // translation     
		T_right_FTsensor_elbow(0,3) = -0.00;
		T_right_FTsensor_elbow(1,3) = -0.15228+0.068;
		T_right_FTsensor_elbow(2,3) =  0.0;
		    // rotation
		T_right_FTsensor_elbow(0,0) = -1.0;
		T_right_FTsensor_elbow(1,2) =  1.0;
		T_right_FTsensor_elbow(2,1) =  1.0;

		// Homogeneous Transformations of the arms FT sensors frames wrt. the root frame
		// Eigen::MatrixXd T_left_ArmFTsensor_root(4,4),
		//                 T_right_ArmFTsensor_root(4,4);

		// compute the homogeneous transformations
		T_left_ArmFTsensor_root  = T_left_elbow_root  * T_left_FTsensor_elbow;
		T_right_ArmFTsensor_root = T_right_elbow_root * T_right_FTsensor_elbow;

		// extracting the rotation matrices
		Rot_left_ArmFTsensor_root  = T_left_ArmFTsensor_root.block(0,0, 3,3);
		Rot_right_ArmFTsensor_root = T_right_ArmFTsensor_root.block(0,0, 3,3);


	}

	void ComputeWrenchTransforms( iCub::iKin::iKinChain *left_elbow_chain,
								  iCub::iKin::iKinChain *right_elbow_chain)
	{
		
		// computing first the homogeneous transformations
		ComputeHomogeneousTransforms( left_elbow_chain,
									  right_elbow_chain);

		// Force/Torque transformation matrices
	    // skew symmetric transformation matrices
		Eigen::MatrixXd SkewMx_left_ArmFTsensor_root(3,3),
		                SkewMx_right_ArmFTsensor_root(3,3);

		SkewMx_left_ArmFTsensor_root.setZero(3,3);
		SkewMx_left_ArmFTsensor_root(0,1) = -T_left_ArmFTsensor_root(2,3);
		SkewMx_left_ArmFTsensor_root(0,2) =  T_left_ArmFTsensor_root(1,3);
		SkewMx_left_ArmFTsensor_root(1,0) =  T_left_ArmFTsensor_root(2,3);
		SkewMx_left_ArmFTsensor_root(1,2) = -T_left_ArmFTsensor_root(0,3);
		SkewMx_left_ArmFTsensor_root(2,0) = -T_left_ArmFTsensor_root(1,3);
		SkewMx_left_ArmFTsensor_root(2,1) =  T_left_ArmFTsensor_root(0,3);

		SkewMx_right_ArmFTsensor_root.setZero(3,3);
		SkewMx_right_ArmFTsensor_root(0,1) = -T_right_ArmFTsensor_root(2,3);
		SkewMx_right_ArmFTsensor_root(0,2) =  T_right_ArmFTsensor_root(1,3);
		SkewMx_right_ArmFTsensor_root(1,0) =  T_right_ArmFTsensor_root(2,3);
		SkewMx_right_ArmFTsensor_root(1,2) = -T_right_ArmFTsensor_root(0,3);
		SkewMx_right_ArmFTsensor_root(2,0) = -T_right_ArmFTsensor_root(1,3);
		SkewMx_right_ArmFTsensor_root(2,1) =  T_right_ArmFTsensor_root(0,3);

		//cout << " T_right_ArmFTsensor_root is : \n"<< T_right_ArmFTsensor_root << endl;

		// wrench transformation matrices

		    // left arm
		WrenchTrsf_left_ArmFTsensor_root.block(0,0, 3,3) = T_left_ArmFTsensor_root.block(0,0, 3,3);
		WrenchTrsf_left_ArmFTsensor_root.block(3,0, 3,3) = SkewMx_left_ArmFTsensor_root * T_left_ArmFTsensor_root.block(0,0, 3,3);
		WrenchTrsf_left_ArmFTsensor_root.block(3,3, 3,3) = T_left_ArmFTsensor_root.block(0,0, 3,3);
		
		    // right arm
		WrenchTrsf_right_ArmFTsensor_root.block(0,0, 3,3) = T_right_ArmFTsensor_root.block(0,0, 3,3);
		WrenchTrsf_right_ArmFTsensor_root.block(3,0, 3,3) = SkewMx_right_ArmFTsensor_root * T_right_ArmFTsensor_root.block(0,0, 3,3);
		WrenchTrsf_right_ArmFTsensor_root.block(3,3, 3,3) = T_right_ArmFTsensor_root.block(0,0, 3,3);
		

	}


};


#endif // ArmsForceTorqueKinChain_H