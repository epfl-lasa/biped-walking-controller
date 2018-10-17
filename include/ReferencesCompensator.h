#ifndef ReferencesCompensator_H
#define ReferencesCompensator_H

#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>


#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

//#include "Discrete_CP_LIP_Model.h"
// #include "CpStanceFootPose.h"
#include "CpMath_Utilities.h"

#include "TemplateModels.h"


using namespace yarp::sig;
using namespace iCub::ctrl;

using namespace std;
using namespace Eigen;

// class CP_LIP_Model
class ReferencesCompensator
{

    // Vector3d d_t_c;
    // yarp::sig::Vector d_AxisAngle_c;
    // Matrix3d Skew_Mu;
    // Matrix3d L_Mu_Theta;

    VectorXd m_acceleration_init;
    VectorXd est_velocity_base;

    public : 

        ReferencesCompensator();

        ~ReferencesCompensator();

        // // Jacobian associated with the configuration error
        // MatrixXd Jac_Mu_Theta;

        // Vector of feature error
        VectorXd d_eta_c;

        // Velocity Twist Matrix associated to the robot base
        MatrixXd BaseVelocityTwist_left;

        MatrixXd BaseVelocityTwist_right;

        // Compensation task Jacobian of the stance leg in the base
        MatrixXd CompTaskJacobianStanceLeg;

        MatrixXd CompTaskJacobianLeftLeg;
        MatrixXd CompTaskJacobianRightLeg;

        // gain vector of the servoing
        VectorXd ServoingGain;

        VectorXd q_dot_left_init;
        VectorXd q_dot_left;

        VectorXd q_dot_right_init;
        VectorXd q_dot_right;


        MatrixXd getJacobianForAxisAngle(MatrixXd d_H_c);

        
        // Vector of feature error
        VectorXd FeatureError_d_H_c(MatrixXd d_H_c);

        // Velocity Twist Matrix associated to the robot base
        MatrixXd getBaseVelocityTwist_left(iCub::iKin::iKinChain *LeftLegChain);

        MatrixXd getBaseVelocityTwist_right(iCub::iKin::iKinChain *RightLegChain);

        // The Compensation task Jacobian of the stance leg in the base
        MatrixXd getCompTaskJacobianStanceLeg(iCub::iKin::iKinChain *LeftLegChain, iCub::iKin::iKinChain *RightLegChain, bool left_stance);

        MatrixXd getCompTaskJacobianLeftLeg(iCub::iKin::iKinChain *LeftLegChain);

        MatrixXd getCompTaskJacobianRightLeg(iCub::iKin::iKinChain *RightLegChain);

        // Set the servoing gain
        void setservoingGain(VectorXd sGain);

        VectorXd getCompensated_lljoints(MatrixXd d_H_c, iCub::iKin::iKinChain *LeftLegChain, double SamplingTime);

        VectorXd getCompensated_rljoints(MatrixXd d_H_c, iCub::iKin::iKinChain *RightLegChain, double SamplingTime);

        // ========== COM ===================

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

        VectorXd getEstimatedCoMVelocity(double T, VectorXd m_acceleration, Vector m_orientation_rpy);

        VectorXd get_m_Acceleration_InBase (VectorXd m_acceleration, Vector m_orientation_rpy);

        void ComputeComError(   Discrete_CP_LIP_Model *DMod,
                                CpStanceFootPose *CoPref,
                                Vector left_Leg_Pose,
                                Vector right_Leg_Pose,
                                VectorXd m_CoM_leftF,
                                VectorXd CoM_measurements_offset,
                                VectorXd m_CoM_velocity, int CpModelOption, bool left_stance,
                                VectorXd l_foot_FT_vector,
                                VectorXd r_foot_FT_vector);

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

 
#endif // ReferencesCompensator



