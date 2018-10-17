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

// 4. EstimatorCompensators
// 	4.1. InertialCompensator
// 	4.2. StatesToInputCompemsator
// 	4.3. ReferencesCompensator
// 	4.4. CoMStatesEstimator

#ifndef EstimatorCompensators_H
#define EstimatorCompensators_H


#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include "PatternsGenerator.h"
#include "TemplateModels.h"
#include "CpMath_Utilities.h"

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;

using namespace std;
using namespace Eigen;



class InertialCompensator
{
    
    VectorXd TrslBaseCoM;      // Translation vector from base to CoM

    public :

        MatrixXd DesTrsfRealStanceFootInHorBase;   // 
        MatrixXd CurTrsfRealStanceFootInHorBase;
        MatrixXd ReferenceTrsfHorFootInHorBase;
        MatrixXd TrsfCurBaseInWorldIMU;
              
        MatrixXd FwKRightFootInBase;
        MatrixXd FwKLeftFootInBase;

        Matrix3d CurRotRealStFootInHorBase;
        Matrix3d RotRealStFootInHorStFoot;

        MatrixXd ReferenceTrsfSwingFootInHorBase;
        MatrixXd DesTrsfSwingFootInCurBase;

        MatrixXd DesTrsfLeftFootInHorBase;
        MatrixXd DesTrsfRightFootInHorBase;

        // Transformation of Homogeoneous transformation into Pose vector with axis/angle representation of orientation
        yarp::sig::Vector DesiredLeftLegPoseAsAxisAngles;
        yarp::sig::Vector DesiredRightLegPoseAsAxisAngles;

        yarp::sig::Vector DesiredLeftLegAngleAxisOrientation;
        yarp::sig::Vector DesiredRightLegAngleAxisOrientation;

        yarp::sig::Matrix DesiredRotationMatrixLeftLeg;
        yarp::sig::Matrix DesiredRotationMatrixRightLeg;

        // Methods

        InertialCompensator();

        ~InertialCompensator();

        void getDesiredFeetInCorrectedBase(                   int SptFt[],
                                                yarp::sig::Vector LLegPose,
                                                yarp::sig::Vector RLegPose,
                                                yarp::sig::Vector m_orientation_rpy,
                                            CpGaitTransformations *GTj);

        // void getDesiredFeetInCorrectedBase(                   int SptFt[],
        //                                         yarp::sig::Vector LLegPose,
        //                                         yarp::sig::Vector RLegPose,
        //                                         Eigen::VectorXd Inertial_measurements,
        //                                     CpGaitTransformations *GTj);

        void SetImuTransformation(yarp::sig::Vector IMU_RollPitchYaw);

        Vector3d getEulerAnglesXYZ_FixedFrame(Matrix3d CurRotRealStFootInHorBase);

        Matrix3d getComposedOrientFixedFrame(Vector3d OrientXY_F_hF);
        							
        //MatrixXd getDesiredSwingFootInCurBase();
        void SetImuTransformation();
        					 
        void SetTranslationBaseCoM(VectorXd t_B_CoM);

        void CompensateTransformsWithIMU(CpGaitTransformations *GTj,
                                         yarp::sig::Vector lleg_EE_pose,
                                         yarp::sig::Vector rleg_EE_pose,
                                         yarp::sig::Vector ImuOrientationRYP,
                                                       int Stance[],
                                                      bool isActive);
        
};

//#endif // InertialCompensator_H


// ===========================================================================================



class StatesToInputCompensator
{
	// ----------------- Force Torque feedback -----------------------------------------------
	public:
		bool useAdmittanceLaw;

		//
		int mysign(double Val)
		{
		    return (0.0 < Val) - (Val < 0.0);
		}

		double VxComFeedback;
		double VyComFeedback;
		double WzComFeedback;

		// threshold
		double pveVxThreshold;		// threshold for positive Vx
		double nveVxThreshold;		// threshold for negative Vx
		double pveVyThreshold;		// threshold for positive Vy
		double nveVyThreshold;		// threshold for negative Vy
		double pveWzThreshold;		// threshold for positive Wz
		double nveWzThreshold;		// threshold for negative Wz

        //
        double gain_x;
        double gain_y;
        double gain_R;
        //

		StatesToInputCompensator();
		~StatesToInputCompensator();

		KalmanFilter *KF_VeloEstimatorX;
		KalmanFilter *KF_VeloEstimatorY;
		KalmanFilter *KF_VeloEstimatorR;

        // force/velocity dynamics
        DynamicSystemSolver X_motionDyn;
        DynamicSystemSolver Y_motionDyn;
        DynamicSystemSolver R_motionDyn;

		void InitS2ICompensator(double dt, VectorXd FdckThreshold);
 
		void ComputeS2IFeedback( int TypeOfFeedback, 
                                VectorXd EstimatedFeetForces, 
                                VectorXd EstimatedArmsForce, 
                                VectorXd arm_FT_offset);

		void SetThreshold(VectorXd _Thrld);

};

// #endif // StatesToInputCompensator_H


// // ====================================================================================

// // class CP_LIP_Model
// class ReferencesCompensator
// {

//     // Vector3d d_t_c;
//     // yarp::sig::Vector d_AxisAngle_c;
//     // Matrix3d Skew_Mu;
//     // Matrix3d L_Mu_Theta;

//     VectorXd m_acceleration_init;
//     VectorXd est_velocity_base;

//     public : 

//         ReferencesCompensator();

//         ~ReferencesCompensator();

//         // // Jacobian associated with the configuration error
//         // MatrixXd Jac_Mu_Theta;

//         // Vector of feature error
//         VectorXd d_eta_c;

//         // Velocity Twist Matrix associated to the robot base
//         MatrixXd BaseVelocityTwist_left;

//         MatrixXd BaseVelocityTwist_right;

//         // Compensation task Jacobian of the stance leg in the base
//         MatrixXd CompTaskJacobianStanceLeg;

//         MatrixXd CompTaskJacobianLeftLeg;
//         MatrixXd CompTaskJacobianRightLeg;

//         // gain vector of the servoing
//         VectorXd ServoingGain;

//         VectorXd q_dot_left_init;
//         VectorXd q_dot_left;

//         VectorXd q_dot_right_init;
//         VectorXd q_dot_right;


//         MatrixXd getJacobianForAxisAngle(MatrixXd d_H_c);

        
//         // Vector of feature error
//         VectorXd FeatureError_d_H_c(MatrixXd d_H_c);

//         // Velocity Twist Matrix associated to the robot base
//         MatrixXd getBaseVelocityTwist_left(iCub::iKin::iKinChain *LeftLegChain);

//         MatrixXd getBaseVelocityTwist_right(iCub::iKin::iKinChain *RightLegChain);

//         // The Compensation task Jacobian of the stance leg in the base
//         MatrixXd getCompTaskJacobianStanceLeg(iCub::iKin::iKinChain *LeftLegChain, iCub::iKin::iKinChain *RightLegChain, bool left_stance);

//         MatrixXd getCompTaskJacobianLeftLeg(iCub::iKin::iKinChain *LeftLegChain);

//         MatrixXd getCompTaskJacobianRightLeg(iCub::iKin::iKinChain *RightLegChain);

//         // Set the servoing gain
//         void setservoingGain(VectorXd sGain);

//         VectorXd getCompensated_lljoints(MatrixXd d_H_c, iCub::iKin::iKinChain *LeftLegChain, double SamplingTime);

//         VectorXd getCompensated_rljoints(MatrixXd d_H_c, iCub::iKin::iKinChain *RightLegChain, double SamplingTime);

//         // ========== COM ===================

//         VectorXd    trsl_com_GBase;
//         // vector of CoM states error
//         VectorXd    m_CoM_left_Foot;
//         VectorXd    m_CoM_right_Foot;
//         VectorXd    CoM_position_error;
//         VectorXd    CoM_velocity_error;
//         VectorXd    Model_statesX_error;
//         VectorXd    Model_statesY_error;

//         VectorXd    Model_statesR_error;

//         VectorXd    m_acceleration_in_Body;

//         VectorXd getEstimatedCoMVelocity(double T, VectorXd m_acceleration, Vector m_orientation_rpy);

//         VectorXd get_m_Acceleration_InBase (VectorXd m_acceleration, Vector m_orientation_rpy);

//         void ComputeComError(   Discrete_CP_LIP_Model *DMod,
//                                 CpStanceFootPose *CoPref,
//                                 Vector left_Leg_Pose,
//                                 Vector right_Leg_Pose,
//                                 VectorXd m_CoM_leftF,
//                                 VectorXd CoM_measurements_offset,
//                                 VectorXd m_CoM_velocity, int CpModelOption, bool left_stance,
//                                 VectorXd l_foot_FT_vector,
//                                 VectorXd r_foot_FT_vector);

//         VectorXd getCoM_in_GlobalBase();

//         VectorXd getCoM_PositionError();

//         VectorXd getCoM_VelocityError();

//         VectorXd get_ModelStatesErrorX();

//         VectorXd get_ModelStatesErrorY();

//         VectorXd get_ModelStatesErrorR();


//         // CoM velocity of the template model relative to the stance foot
//         VectorXd V_com_sft_inertial;  // expressed in the inertial frame
//         VectorXd V_com_sft;           // expressed in the stance foot frame

//         firstOrderIntegrator *FilterVeloCoM_sft;
//         firstOrderIntegrator *FilterVeloCoM_sft2;

//         // states observation
//         Eigen::MatrixXd  A_Obs;
//         Eigen::VectorXd B_Obs;

//         Eigen::MatrixXd  Ar_Obs;
//         Eigen::VectorXd Br_Obs;

//         Eigen::VectorXd StatesX_Obs;
//         Eigen::VectorXd StatesY_Obs;

//         Eigen::VectorXd ObsGain;

//         double m_xZMP;
//         double m_yZMP;
//         double m_TauZ;
//         double m_disFx;
//         double m_disFy;

//         double K_Wdot;

//         double WlkMode;

//         double PrevStanceX;
//         double PrevStanceY;

//         double DeltaStanceX;
//         double DeltaStanceY;

//         void InitializeObserver(Discrete_CP_LIP_Model *DMod);

//         // States estimation
//         Eigen::MatrixXd A_Obs3;
//         Eigen::VectorXd B_Obs3;
//         Eigen::RowVectorXd C_Obs3;


//         Eigen::VectorXd StatesX_Obs3;
//         Eigen::VectorXd StatesY_Obs3;
//         Eigen::VectorXd GainObs_3;

//         // Dynamics solver
//         DynamicSystemSolver X_motion;
//         DynamicSystemSolver Y_motion;
//         DynamicSystemSolver R_motion;

//         // ZMP based Dynamic Filter for the CoM
//         ZmpDynamicFilter DynFilterZmpCoM;

//         // ZMP errors
//         double Delta_xZmp;
//         double Delta_yZmp;

//         double get_xZmpError();
//         double get_yZmpError();
    

// };


// =========================================================================================================

// =========================================================================================================

class CoMStatesEstimator
{
	public:

		VectorXd xComEstimated;
		VectorXd yComEstimated;
		VectorXd ThetazEstimated;

        VectorXd    trsl_com_GBase;
        // vector of CoM states error
        VectorXd    m_CoM_left_Foot;
        VectorXd    m_CoM_right_Foot;
        VectorXd    CoM_position_error;
        VectorXd    CoM_velocity_error;
        VectorXd    Model_statesX_error;
        VectorXd    Model_statesY_error;
        VectorXd    Model_statesR_error;

        VectorXd    m_acceleration_in_Body;


		CoMStatesEstimator();
		~CoMStatesEstimator();

		firstOrderIntegrator *Filter_CoM_Velo;

		void InitComEstimator(Discrete_CP_LIP_Model *DMod);

        // ComputeComError
		void EstimateStates(Discrete_CP_LIP_Model *DMod,
	                        CpStanceFootPose *CoPref, 
	                        Vector left_Leg_Pose,
	                        Vector right_Leg_Pose,
	                        VectorXd m_CoM_leftF,
	                        VectorXd CoM_measurements_offset, 
	                        VectorXd m_CoM_velocity, int CpModelOption, bool left_stance,
	                        VectorXd l_foot_FT_vector,
	                        VectorXd r_foot_FT_vector);

		// void EstimateError();

		// VectorXd getPosition();
		// VectorXd getVelocity();
		// VectorXd getPositionError();
		// VectorXd getVelocityError();

        //VectorXd getEstimatedCoMVelocity(double T, VectorXd m_acceleration, Vector m_orientation_rpy);

        VectorXd get_m_Acceleration_InBase (VectorXd m_acceleration, Vector m_orientation_rpy);

         VectorXd getCoM_in_GlobalBase();

        VectorXd getCoM_PositionError();

        VectorXd getCoM_VelocityError();

        VectorXd get_ModelStatesErrorX();

        VectorXd get_ModelStatesErrorY();

        VectorXd get_ModelStatesErrorR();


        // CoM velocity of the template model relative to the stance foot
        VectorXd V_com_sft_inertial;  // expressed in the inertial frame
        VectorXd V_com_sft;           // expressed in the stance foot frame

        firstOrderIntegrator *FilterVeloCoM_sft;
        firstOrderIntegrator *FilterVeloCoM_sft2;

        // states observation
        Eigen::MatrixXd  A_Obs;
        Eigen::VectorXd B_Obs;

        Eigen::MatrixXd  Ar_Obs;
        Eigen::VectorXd Br_Obs;

        Eigen::VectorXd StatesX_Obs;
        Eigen::VectorXd StatesY_Obs;

        Eigen::VectorXd ObsGain;

        double m_xZMP;
        double m_yZMP;
        double m_TauZ;
        double m_disFx;
        double m_disFy;

        double K_Wdot;

        double WlkMode;

        double PrevStanceX;
        double PrevStanceY;

        double DeltaStanceX;
        double DeltaStanceY;

        void InitializeObserver(Discrete_CP_LIP_Model *DMod);

        // States estimation
        Eigen::MatrixXd A_Obs3;
        Eigen::VectorXd B_Obs3;
        Eigen::RowVectorXd C_Obs3;


        Eigen::VectorXd StatesX_Obs3;
        Eigen::VectorXd StatesY_Obs3;
        Eigen::VectorXd GainObs_3;

        // Dynamics solver
        DynamicSystemSolver X_motion;
        DynamicSystemSolver Y_motion;
        DynamicSystemSolver R_motion;

        // ZMP based Dynamic Filter for the CoM
        ZmpDynamicFilter DynFilterZmpCoM;

        // ZMP errors
        double Delta_xZmp;
        double Delta_yZmp;

        double get_xZmpError();
        double get_yZmpError();

};

#endif //EstimatorCompensators_H

