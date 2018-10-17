#ifndef ArmsForceTorqueSensorsChains_H
#define ArmsForceTorqueSensorsChains_H

#include <string>
#include <iostream>
#include <cmath>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "SubLimbsUpperBody.h"
#include "CpMath_Utilities.h"


using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace std;
using namespace Eigen;

class ArmsForceTorqueSensorsChains
{	


		

	public :

		// // instantiating sub limbs objects 
	    subLimb_LeftArm  root_left_yawShoulderframe;
	    subLimb_RightArm root_right_yawShoulderframe;

		iCub::iKin::iKinChain *left_yawShoulder_chain;
    	iCub::iKin::iKinChain *right_yawShoulder_chain;

    	yarp::sig::Vector left_subchain_jts;
	    yarp::sig::Vector right_subchain_jts;




		// Homogenous transformation
		Eigen::MatrixXd root_Transform_leftArmFTsensor;
		Eigen::MatrixXd root_Transform_rightArmFTsensor;

		// Rotation matrix
		Eigen::MatrixXd root_Rotation_leftArmFTsensor;
		Eigen::MatrixXd root_Rotation_rightArmFTsensor;

		// Positon
		Eigen::MatrixXd root_Position_leftArmFTsensor;
		Eigen::MatrixXd root_Position_rightArmFTsensor;

		// Wrench transformation
		Eigen::MatrixXd root_WrenchTransf_leftArmFTsensor;
		Eigen::MatrixXd root_WrenchTransf_rightArmFTsensor;

		// postion of the hand CoMs in the force torque sensor frame
		Eigen::VectorXd larmFT_sensor_t_lhand_com;
		Eigen::VectorXd rarmFT_sensor_t_rhand_com;

		// postion of the forearm CoMs in the force torque sensor frame
		Eigen::VectorXd larmFT_sensor_t_lforearm_com;
		Eigen::VectorXd rarmFT_sensor_t_rforearm_com;

		// postion of the upper arm CoMs in the force torque sensor frame
		Eigen::VectorXd larmFT_sensor_t_lupperarm_com;
		Eigen::VectorXd rarmFT_sensor_t_rupperarm_com;

		// position of the forearm com in the prosup frame.
		Eigen::VectorXd lhandframe_t_lhand_com;
		Eigen::VectorXd rhandframe_t_rhand_com;

		// position of the forearm com in the prosup frame.
		Eigen::VectorXd laprosupframe_t_lforearm_com;
		Eigen::VectorXd raprosupframe_t_rforearm_com;

		// Wrenches of the arms links
		Eigen::VectorXd left_handWrench_sensor;
		Eigen::VectorXd left_forearmWrench_sensor;
		Eigen::VectorXd left_upperarmWrench_sensor;
		Eigen::VectorXd left_armTotalWrench_sensor;

		Eigen::VectorXd right_handWrench_sensor;
		Eigen::VectorXd right_forearmWrench_sensor;
		Eigen::VectorXd right_upperarmWrench_sensor;
		Eigen::VectorXd right_armTotalWrench_sensor;

		Transformations Transforms;

		// arm links mass
		double l_hand_mass;
		double l_forearm_mass;
		double l_upperarm_mass;
		
		double r_hand_mass;
		double r_forearm_mass;
		double r_upperarm_mass;


		// position factor 
		double forearm_factor;
		double upperarm_factor;

		// DH parameters of the transformation from the elbow to Shoulder 3 (frame 7 to frame 6)
		double 	DH_Left_Elbow2yawShoulder[4];
		double 	DH_Right_Elbow2yawShoulder[4];
		double 	DH_Left_wristprosup2Elbow[4];
		double 	DH_Right_wristprosup2Elbow[4];

		// 
		ArmsForceTorqueSensorsChains() {}

		~ArmsForceTorqueSensorsChains()
		{
			// if(left_yawShoulder_chain)
			// {
			// 	delete left_yawShoulder_chain;
			// 	left_yawShoulder_chain = 0;
			// }

			// if(right_yawShoulder_chain)
			// {
			// 	delete right_yawShoulder_chain;
			// 	right_yawShoulder_chain = 0;
			// }

		}


		void Init() 
		{
			
			// position factor 
			double foream_factor   = 0.40;
			double upperarm_factor = 0.33;  // to be checked

			// DH parameters of the transformation from the elbow to Shoulder 3 (frame 7 to frame 6)
			// left hand
					DH_Left_Elbow2yawShoulder[0] = -0.015;					// ai : translation along the 
					DH_Left_Elbow2yawShoulder[1] = 0.0;						// di : translation along the Zn-1 axis
					DH_Left_Elbow2yawShoulder[2] = M_PI/2.0;					// alphai : the twist between axes
					DH_Left_Elbow2yawShoulder[3] = 0.0;						// offset theta

			// left hand
					DH_Right_Elbow2yawShoulder[0] = 0.015;					// ai : translation along the 
					DH_Right_Elbow2yawShoulder[1] = 0.0;						// di : translation along the Zn-1 axis
					DH_Right_Elbow2yawShoulder[2] = M_PI/2.0;					// alphai : the twist between axes
					DH_Right_Elbow2yawShoulder[3] = 0.0;						// offset theta

			// DH parameters of the transformation from wrist prosup to elbow 
			// left hand
					DH_Left_wristprosup2Elbow[0] = 0.00;					// ai : translation along the 
					DH_Left_wristprosup2Elbow[1] = 0.1413;					// di : translation along the Zn-1 axis
					DH_Left_wristprosup2Elbow[2] =  M_PI/2.0;				// alphai : the twist between axes
					DH_Left_wristprosup2Elbow[3] = -M_PI/2.0; 

			// left hand (frame 8 end of the forearm  to frame 7 elbow)
					DH_Right_wristprosup2Elbow[0] =  0.0;					// ai : translation along the 
					DH_Right_wristprosup2Elbow[1] = -0.1413;				// di : translation along the Zn-1 axis
					DH_Right_wristprosup2Elbow[2] =  M_PI/2.0;				// alphai : the twist between axes
					DH_Right_wristprosup2Elbow[3] = -M_PI/2.0;

			// transformation
			Transforms.Init();

			root_Transform_leftArmFTsensor.resize(4,4);
			root_Transform_rightArmFTsensor.resize(4,4);

			root_Transform_leftArmFTsensor(3,3) = 1.0;
			root_Transform_rightArmFTsensor(3,3) = 1.0;

			root_Rotation_leftArmFTsensor.resize(3,3);
			root_Rotation_rightArmFTsensor.resize(3,3);

			root_WrenchTransf_leftArmFTsensor.resize(6,6);
			root_WrenchTransf_rightArmFTsensor.resize(6,6);

			root_WrenchTransf_leftArmFTsensor.setIdentity(6,6);
			root_WrenchTransf_rightArmFTsensor.setIdentity(6,6);

			left_yawShoulder_chain  = new iCub::iKin::iKinLimb();
        	right_yawShoulder_chain = new iCub::iKin::iKinLimb();

	        left_yawShoulder_chain  = root_left_yawShoulderframe.asChain();
	        right_yawShoulder_chain = root_right_yawShoulderframe.asChain();

	        left_subchain_jts.resize(left_yawShoulder_chain->getDOF());
	        right_subchain_jts.resize(right_yawShoulder_chain->getDOF());

	        // wrenches of the arms links
	        left_handWrench_sensor.resize(6);
			left_forearmWrench_sensor.resize(6);
			left_upperarmWrench_sensor.resize(6);
			left_armTotalWrench_sensor.resize(6);

			right_handWrench_sensor.resize(6);
			right_forearmWrench_sensor.resize(6);
			right_upperarmWrench_sensor.resize(6);
			right_armTotalWrench_sensor.resize(6);

	        // postion of the hand CoM in the force torque sensor frame
			larmFT_sensor_t_lhand_com.resize(3);
			rarmFT_sensor_t_rhand_com.resize(3);

			// postion of the forearm CoMs in the force torque sensor frame
			larmFT_sensor_t_lforearm_com.resize(3);
			rarmFT_sensor_t_rforearm_com.resize(3);

			// postion of the upper arm CoMs in the force torque sensor frame
			larmFT_sensor_t_lupperarm_com.resize(3);
			rarmFT_sensor_t_rupperarm_com.resize(3);

				larmFT_sensor_t_lupperarm_com(0) = 0.0;
				larmFT_sensor_t_lupperarm_com(1) = 0.0;
				larmFT_sensor_t_lupperarm_com(2) = 0.02;

				rarmFT_sensor_t_rupperarm_com(0) = 0.0;
				rarmFT_sensor_t_rupperarm_com(1) = 0.0;
				rarmFT_sensor_t_rupperarm_com(2) = 0.02;

			// position of the forearm com in the prosup frame.
			lhandframe_t_lhand_com.resize(3);
			rhandframe_t_rhand_com.resize(3);

				lhandframe_t_lhand_com(0) = 0.0;
				lhandframe_t_lhand_com(1) = 0.0;
				lhandframe_t_lhand_com(2) = 0.0;

				rhandframe_t_rhand_com(0) = 0.0;
				rhandframe_t_rhand_com(1) = 0.0;
				rhandframe_t_rhand_com(2) = 0.0;

			// position of the forearm com in the prosup frame.
			laprosupframe_t_lforearm_com.resize(3);
			raprosupframe_t_rforearm_com.resize(3);

				// left arm
				laprosupframe_t_lforearm_com(0) =  0.0;
				laprosupframe_t_lforearm_com(1) = -0.1413 * forearm_factor;
				laprosupframe_t_lforearm_com(2) =  0.0;
				
				//right arm
				raprosupframe_t_rforearm_com(0) =  0.0;
				raprosupframe_t_rforearm_com(1) =  0.1413 * forearm_factor;
				raprosupframe_t_rforearm_com(2) =  0.0;

			// link masses
			l_hand_mass = 0.213;
			l_forearm_mass = 0.525 + 0.1;
			l_upperarm_mass = 0.727843 + 0.1;
						
			r_hand_mass = 0.213;
			r_forearm_mass = 0.525 + 0.1;
			r_upperarm_mass = 0.727843 + 0.1;

			
		}

		bool updateArmsSubChains(iCub::iKin::iKinChain *Left_Arm_Chain, iCub::iKin::iKinChain *Right_Arm_Chain)
		{
			//
			
			yarp::sig::Vector left_jts_values  = Left_Arm_Chain->getAng();
			yarp::sig::Vector right_jts_values = Right_Arm_Chain->getAng();
			// 
	        left_subchain_jts[0] = left_jts_values[0];
	        left_subchain_jts[1] = left_jts_values[1];
	        left_subchain_jts[2] = left_jts_values[2];

	        right_subchain_jts[0] = right_jts_values[0];
	        right_subchain_jts[1] = right_jts_values[1];
	        right_subchain_jts[2] = right_jts_values[2];

	        // left_yawShoulder_chain->setAng(CTRL_DEG2RAD * left_subchain_jts);
	        // right_yawShoulder_chain->setAng(CTRL_DEG2RAD * right_subchain_jts);

	        left_yawShoulder_chain->setAng(left_subchain_jts);
	        right_yawShoulder_chain->setAng(right_subchain_jts);

	       return true;

		}

		// compute the DH matrix between two successife frame
		Eigen::MatrixXd compute_DH_transform(double dh_[], double thetai)
		{
			Eigen::MatrixXd DH_matrix(4,4);
			DH_matrix.setZero();
			DH_matrix(3,3) = 1.0;

			DH_matrix(0,0) = cos(thetai+dh_[3]); 	DH_matrix(0,1) = -sin(thetai+dh_[3])*cos(dh_[2]); 	DH_matrix(0,2) = sin(thetai+dh_[3])*sin(dh_[2]);	DH_matrix(0,3) = dh_[0] * cos(thetai+dh_[3]); 	
			DH_matrix(1,0) = sin(thetai+dh_[3]); 	DH_matrix(1,1) =  cos(thetai+dh_[3])*cos(dh_[2]); 	DH_matrix(1,2) = -cos(thetai+dh_[3])*sin(dh_[2]); 	DH_matrix(1,3) = dh_[0] * sin(thetai+dh_[3]); 
													DH_matrix(2,1) = sin(dh_[2]); 						DH_matrix(2,2) =  cos(dh_[2]); 						DH_matrix(2,3) = dh_[1];

			return DH_matrix;

		}

		// computation of link's compensation wrench to apply to the force torque measurements in the sensor frame 
		Eigen::VectorXd compute_compensationWrench(double link_mass, Eigen::MatrixXd s_R_w, Eigen::VectorXd s_t_linkcom)
		{
			// force transformation matrix from world to sensor
			Eigen::MatrixXd G_force(6,3);
			G_force.setZero();
			G_force.block<3,3>(0,0) = s_R_w;
			G_force.block<3,3>(3,0) = Transforms.ComputeSkewSymmetricMatrix(s_t_linkcom) * s_R_w;

			// estimation of the link weight
			Eigen::VectorXd estim_weight(3);

			estim_weight(0) = 0.0;
			estim_weight(1) = 0.0;
			estim_weight(2) = -link_mass * 9.81;

			return G_force * estim_weight;
		}

		bool computeTransformFTsensor2root()
		{
				
			// homogenous transformation from sensor to shoulder3 frame
			Eigen::MatrixXd layawShoulder_H_larmFT_sensor(4,4);
			Eigen::MatrixXd rayawShoulder_H_rarmFT_sensor(4,4);

			// left hand
			layawShoulder_H_larmFT_sensor.setZero();		layawShoulder_H_larmFT_sensor(3,3) = 1.0;
		    // translation     
				// layawShoulder_H_larmFT_sensor(0,3) = 0.015;
				// layawShoulder_H_larmFT_sensor(1,3) = 0.0;
				// layawShoulder_H_larmFT_sensor(2,3) = 0.068;
				//     // rotation
				// layawShoulder_H_larmFT_sensor(1,0) =  1.0;
				// layawShoulder_H_larmFT_sensor(0,1) = -1.0;
				// layawShoulder_H_larmFT_sensor(2,2) =  1.0;

			// right hand
			rayawShoulder_H_rarmFT_sensor.setZero();		rayawShoulder_H_rarmFT_sensor(3,3) = 1.0;
			// translation     
				// rayawShoulder_H_rarmFT_sensor(0,3) = 0.015;
				// rayawShoulder_H_rarmFT_sensor(1,3) = 0.0;
				// rayawShoulder_H_rarmFT_sensor(2,3) = -0.068;
				//     // rotation
				// rayawShoulder_H_rarmFT_sensor(1,0) =  1.0;
				// rayawShoulder_H_rarmFT_sensor(0,1) =  1.0;
				// rayawShoulder_H_rarmFT_sensor(2,2) = -1.0;

			// older

				layawShoulder_H_larmFT_sensor(0,3) = 0.00;
				layawShoulder_H_larmFT_sensor(1,3) = 0.15228-0.068;
				layawShoulder_H_larmFT_sensor(2,3) = 0.0;
				    // rotation
				layawShoulder_H_larmFT_sensor(0,0) =  1.0;
				layawShoulder_H_larmFT_sensor(1,2) = -1.0;
				layawShoulder_H_larmFT_sensor(2,1) =  1.0;

  
			    // translation     
				rayawShoulder_H_rarmFT_sensor(0,3) = -0.00;
				rayawShoulder_H_rarmFT_sensor(1,3) = -0.15228+0.068;
				rayawShoulder_H_rarmFT_sensor(2,3) =  0.0;
				    // rotation
				rayawShoulder_H_rarmFT_sensor(0,0) = -1.0;
				rayawShoulder_H_rarmFT_sensor(1,2) =  1.0;
				rayawShoulder_H_rarmFT_sensor(2,1) =  1.0;

			// left arm transformation from sensor frame to root frame 
			root_Transform_leftArmFTsensor = Transforms.yarpPoseTransform2EigenHmatrix(left_yawShoulder_chain->EndEffPose()) * layawShoulder_H_larmFT_sensor;

			// right arm transformation from sensor frame to root frame
			root_Transform_rightArmFTsensor = Transforms.yarpPoseTransform2EigenHmatrix(right_yawShoulder_chain->EndEffPose()) * rayawShoulder_H_rarmFT_sensor;

			// Extract the rotation and the translation
			root_Rotation_leftArmFTsensor  = root_Transform_leftArmFTsensor.block<3,3>(0,0);
			root_Rotation_rightArmFTsensor = root_Transform_rightArmFTsensor.block<3,3>(0,0);

			root_Position_leftArmFTsensor  = root_Transform_leftArmFTsensor.block<3,1>(0,3);
			root_Position_rightArmFTsensor = root_Transform_rightArmFTsensor.block<3,1>(0,3);

			
			return true;
		}

		bool compute_armsLinksCoMs2FTsensors(iCub::iKin::iKinChain *Left_Arm_Chain, iCub::iKin::iKinChain *Right_Arm_Chain)
		{

			// postion of the hand CoM in the force torque sensor frame
			Eigen::MatrixXd leftArmFTsensor_H_lhand;
			Eigen::MatrixXd rightArmFTsensor_H_rhand;

			leftArmFTsensor_H_lhand = root_Transform_leftArmFTsensor.inverse()  * Transforms.yarpPoseTransform2EigenHmatrix(Left_Arm_Chain->EndEffPose());
			rightArmFTsensor_H_rhand = root_Transform_rightArmFTsensor.inverse() * Transforms.yarpPoseTransform2EigenHmatrix(Right_Arm_Chain->EndEffPose());

			larmFT_sensor_t_lhand_com = leftArmFTsensor_H_lhand.block<3,1>(0,3);
			rarmFT_sensor_t_rhand_com = rightArmFTsensor_H_rhand.block<3,1>(0,3);


			// postion of the forearm CoMs in the force torque sensor frame
				Eigen::MatrixXd layawShoulder_H_larmFT_sensor(4,4);
				Eigen::MatrixXd rayawShoulder_H_rarmFT_sensor(4,4);

				// left hand
				layawShoulder_H_larmFT_sensor.setZero();		layawShoulder_H_larmFT_sensor(3,3) = 1.0;
				    // translation     
					layawShoulder_H_larmFT_sensor(0,3) = 0.015;
					layawShoulder_H_larmFT_sensor(1,3) = 0.0;
					layawShoulder_H_larmFT_sensor(2,3) = 0.068;
					    // rotation
					layawShoulder_H_larmFT_sensor(1,0) =  1.0;
					layawShoulder_H_larmFT_sensor(0,1) = -1.0;
					layawShoulder_H_larmFT_sensor(2,2) =  1.0;

				// right hand
				rayawShoulder_H_rarmFT_sensor.setZero();		rayawShoulder_H_rarmFT_sensor(3,3) = 1.0;
					// translation     
					rayawShoulder_H_rarmFT_sensor(0,3) = 0.015;
					rayawShoulder_H_rarmFT_sensor(1,3) = 0.0;
					rayawShoulder_H_rarmFT_sensor(2,3) = -0.068;
					    // rotation
					rayawShoulder_H_rarmFT_sensor(1,0) =  1.0;
					rayawShoulder_H_rarmFT_sensor(0,1) =  1.0;
					rayawShoulder_H_rarmFT_sensor(2,2) = -1.0;

				// Homogenous transformation from wrist prosup to Shoulder3
				Eigen::MatrixXd layawShoulder_H_lwristprosup(4,4);
				Eigen::MatrixXd rayawShoulder_H_rwristprosup(4,4);

				yarp::sig::Vector left_jts_values  = Left_Arm_Chain->getAng();
				yarp::sig::Vector right_jts_values = Right_Arm_Chain->getAng();

				layawShoulder_H_lwristprosup = compute_DH_transform(DH_Left_Elbow2yawShoulder, left_jts_values[3]) * compute_DH_transform(DH_Left_wristprosup2Elbow, left_jts_values[4]);
				rayawShoulder_H_rwristprosup = compute_DH_transform(DH_Right_Elbow2yawShoulder, right_jts_values[3]) *compute_DH_transform(DH_Right_wristprosup2Elbow, right_jts_values[4]);

				// Homogenous transformation from wrist prosup to arm force torque sensor
				Eigen::MatrixXd larmFT_sensor_H_lawristprosup(4,4);
				Eigen::MatrixXd rarmFT_sensor_H_rawristprosup(4,4);

				larmFT_sensor_H_lawristprosup = layawShoulder_H_larmFT_sensor.inverse() * layawShoulder_H_lwristprosup;
				rarmFT_sensor_H_rawristprosup = rayawShoulder_H_rarmFT_sensor.inverse() * rayawShoulder_H_rwristprosup;
			// left
			larmFT_sensor_t_lforearm_com = larmFT_sensor_H_lawristprosup.block<3,3>(0,0) * laprosupframe_t_lforearm_com;
			// right
			rarmFT_sensor_t_rforearm_com = rarmFT_sensor_H_rawristprosup.block<3,3>(0,0) * raprosupframe_t_rforearm_com;

			// postion of the upper arm CoMs in the force torque sensor frame
			larmFT_sensor_t_lupperarm_com = larmFT_sensor_t_lupperarm_com;
			rarmFT_sensor_t_rupperarm_com = rarmFT_sensor_t_rupperarm_com;

			return true;
		}

		bool EstimateArmsGravityWrenchInSensor(iCub::iKin::iKinChain *Left_Arm_Chain, iCub::iKin::iKinChain *Right_Arm_Chain)
		{
			
			cout << " I am in the Estimate Loop : \n "<<  endl;
			// update the subchain
			if (!updateArmsSubChains(Left_Arm_Chain, Right_Arm_Chain))
			{
				printf("Failed to update the subchain \n");
			}

			// compute the transformation from the arms force torque sensors to the root frame
			if (!computeTransformFTsensor2root())
			{
				printf("Failed to compute the sensor 2 root transformation \n");
			}

			// compute the links coms in the sensor frame
			if (!compute_armsLinksCoMs2FTsensors(Left_Arm_Chain, Right_Arm_Chain))
			{
				printf(" Failed to compute the arms links CoMs in the sensors frame \n");
			}

			

			// left
			left_handWrench_sensor = compute_compensationWrench(l_hand_mass, root_Rotation_leftArmFTsensor.transpose(), larmFT_sensor_t_lhand_com);

			left_forearmWrench_sensor = compute_compensationWrench(l_forearm_mass, root_Rotation_leftArmFTsensor.transpose(), larmFT_sensor_t_lforearm_com);
			
			left_upperarmWrench_sensor = compute_compensationWrench(l_upperarm_mass, root_Rotation_leftArmFTsensor.transpose(), larmFT_sensor_t_lupperarm_com);

			left_armTotalWrench_sensor = left_handWrench_sensor + left_forearmWrench_sensor + left_upperarmWrench_sensor;
			
			// right
			right_handWrench_sensor = compute_compensationWrench(r_hand_mass, root_Rotation_rightArmFTsensor.transpose(), rarmFT_sensor_t_rhand_com);
			right_forearmWrench_sensor = compute_compensationWrench(r_forearm_mass, root_Rotation_rightArmFTsensor.transpose(), rarmFT_sensor_t_rforearm_com);
			right_upperarmWrench_sensor = compute_compensationWrench(r_upperarm_mass, root_Rotation_rightArmFTsensor.transpose(), rarmFT_sensor_t_rupperarm_com);

			right_armTotalWrench_sensor = right_handWrench_sensor + right_forearmWrench_sensor + right_upperarmWrench_sensor;


			return true;
		}


};

#endif //ArmsForceTorqueSensorsChains_H