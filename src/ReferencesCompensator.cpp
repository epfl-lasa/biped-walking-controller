#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <yarp/math/Math.h>

#include "CpMath_Utilities.h"
#include "ReferencesCompensator.h"

using namespace iCub::ctrl;
using namespace yarp::math;


ReferencesCompensator::ReferencesCompensator()
{
    
    // d_t_c.setZero(3);
    // d_AxisAngle_c.setZero(4);
    // d_AxisAngle_c(2) = 1.;
    // Skew_Mu.resize(3,3);
    // Skew_Mu.setZero(3,3);

    // L_Mu_Theta.resize(3,3);
    // L_Mu_Theta.setIdentity(3,3);

    // Jac_Mu_Theta.resize(6,6);
    // Jac_Mu_Theta.setZero(6,6);

    // d_eta_c.resize(6);
    m_acceleration_init.resize(3);
    m_acceleration_init.setZero(3);

    est_velocity_base.resize(3);
    est_velocity_base.setZero(3);

    BaseVelocityTwist_left.resize(6,6);
    BaseVelocityTwist_left.setZero(6,6);

    BaseVelocityTwist_right.resize(6,6);
    BaseVelocityTwist_right.setZero(6,6);

    ServoingGain.resize(6);

    q_dot_left_init.resize(6);
    q_dot_left.resize(6);

    q_dot_right_init.resize(6);
    q_dot_right.resize(6);

    // ====== CoM ==========
    trsl_com_GBase.resize(3);
    trsl_com_GBase.setZero(3);

    m_CoM_left_Foot.resize(3);
    m_CoM_right_Foot.resize(3);

    CoM_position_error.resize(3);
    CoM_position_error.setZero(3);
    CoM_velocity_error.resize(3);
    CoM_velocity_error.setZero(3);
    m_acceleration_in_Body.resize(3);

    V_com_sft_inertial.resize(3);  // expressed in the inertial frame
    V_com_sft.resize(3);
    V_com_sft.setZero(3);

    FilterVeloCoM_sft = new firstOrderIntegrator(0.040, 10.0, 10.0, CoM_velocity_error); //7.5
    FilterVeloCoM_sft2 = new firstOrderIntegrator(0.040, 7.0, 7.0, CoM_velocity_error); //7.5



  StatesX_Obs.resize(2);
  StatesX_Obs.setZero(2);

  StatesY_Obs.resize(2);
  StatesY_Obs.setZero(2);   // to be initialised to the values of the model

  // three states observer
  StatesX_Obs3.resize(3);
  StatesX_Obs3.setZero(3);

  StatesY_Obs3.resize(3);
  StatesY_Obs3.setZero(3);   // to be initialised to the values of the model

  // Observer gains
  ObsGain.resize(2);
  ObsGain(0) = 2.9; //0.2
  ObsGain(1) = 0.8; //1.3

  //
  GainObs_3.resize(3);
  GainObs_3(0) = 0.92;
  GainObs_3(1) = 4.75;
  GainObs_3(2) = 0.4308;

  m_disFx = 0.0;
  m_disFy = 0.0;

  // rotation coefficient
  K_Wdot = 0.4; 

  DeltaStanceX = 0.0;
  DeltaStanceY = 0.0;

  Eigen::MatrixXd Axy(2,2);
  // Axy(0,0) = 0.0;
  // Axy(0,1) = 1.0;
  // Axy(1,0) = -(9.80/0.47)*30.0;
  // Axy(1,1) = -2.* sqrt(-Axy(1,0))*0.707; //-2.*sqrt(9.80/0.47)*30.0* 0.707;//-9.4;  // 6.4
  Axy(0,0) = 0.0;
  Axy(0,1) = 1.0;
  Axy(1,0) = -(9.80/0.47)*30.0;
  Axy(1,1) = -2.* sqrt(-Axy(1,0))*0.707; //-2.*sqrt(9.80/0.47)*30.0* 0.707;//-9.4;  // 6.4

  Eigen::VectorXd Bxy(2);
  Bxy(0) = 0.0;
  Bxy(1) = 1.0;

  Eigen::MatrixXd Arot(2,2), Ad(2,2);
  Arot(0,0) = 0.0;
  Arot(0,1) = 1.0;
  Arot(1,0) = -00.0;
  Arot(1,1) = -10.0;

  Ad(0,0) = 0.0;
  Ad(0,1) = 1.0;
  Ad(1,0) = -00.0;
  Ad(1,1) = -40.0;

  //Eigen::VectorXd Brot(2);

  //X_motion.InitializeDynamics(0.04, Axy, Bxy, StatesX_Obs);
  //Y_motion.InitializeDynamics(0.04, Axy, Bxy, StatesY_Obs);
  R_motion.InitializeDynamics(0.04, Arot, Bxy, StatesX_Obs);
  X_motion.InitializeDynamics(0.04, Ad, Bxy, StatesX_Obs);
  Y_motion.InitializeDynamics(0.04, Ad, Bxy, StatesY_Obs);

}

ReferencesCompensator::~ReferencesCompensator()
{
    if (FilterVeloCoM_sft){
      delete FilterVeloCoM_sft;
      FilterVeloCoM_sft = 0;
    }
    if (FilterVeloCoM_sft2){
      delete FilterVeloCoM_sft2;
      FilterVeloCoM_sft2 = 0;
    }

}

MatrixXd ReferencesCompensator::getJacobianForAxisAngle(MatrixXd d_H_c)
{
  /* This function computes the Jacobian of associated with a rigid transformation
   * represented as homogeneous transformation matrix from the current frame to
   * the desired frame (d_H_c) and where the orientation is represented with an Axis/Angle
   */

  // Jacobian associated with the configuration error
  MatrixXd Jac_Mu_Theta;

    Jac_Mu_Theta.resize(6,6);
    Jac_Mu_Theta.setZero(6,6);


  // extraction of the translation
  Vector3d d_t_c;
  d_t_c.setZero(3);

  d_t_c(0) = d_H_c(0,3);
  d_t_c(1) = d_H_c(1,3);
  d_t_c(2) = d_H_c(2,3);

  // extracrion of the rotation
  yarp::sig::Matrix d_R_c;
  d_R_c.resize(3,3);

  for (int row=0; row<3; row++)
    {
     for (int col=0; col<3; col++)
       {
         d_R_c(row,col) = d_H_c(row,col);
       }
    }



  yarp::sig::Vector d_AxisAngle_c;
  // d_AxisAngle_c.setZero(4);
  // d_AxisAngle_c(2) = 1.;

  d_AxisAngle_c = yarp::math::dcm2axis(d_R_c);

  // function sinc(theta) and sinc(theta/2)
  double sinc_theta, sinc_theta_2;

  sinc_theta   = sin(d_AxisAngle_c(3) + 1e-6)/(d_AxisAngle_c(3) + 1e-6);
  sinc_theta_2 = sin((d_AxisAngle_c(3) + 1e-6)/2.)/((d_AxisAngle_c(3) + 1e-6)/2.);

    
    Matrix3d Skew_Mu;
    Skew_Mu.resize(3,3);
    Skew_Mu.setZero(3,3);

    Skew_Mu(0,1) = -d_AxisAngle_c(2);
    Skew_Mu(0,2) =  d_AxisAngle_c(1);
    Skew_Mu(1,0) =  d_AxisAngle_c(2);
    Skew_Mu(1,2) = -d_AxisAngle_c(0);
    Skew_Mu(2,0) = -d_AxisAngle_c(1);
    Skew_Mu(2,1) =  d_AxisAngle_c(0);

    // Jacobian of the rotation
        Matrix3d L_Mu_Theta;
        L_Mu_Theta.resize(3,3);
        L_Mu_Theta.setIdentity(3,3);

        L_Mu_Theta -= (d_AxisAngle_c(3)/2.)* Skew_Mu * (1-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

    // Building the overall jacobian

        Jac_Mu_Theta.block(0,0, 3,3) = d_H_c.block(0,0, 3,3);
        Jac_Mu_Theta.block(3,3, 3,3) = L_Mu_Theta;

    return Jac_Mu_Theta;

}



// Vector of feature error
VectorXd ReferencesCompensator::FeatureError_d_H_c(MatrixXd d_H_c)
{
    // extraction of the translation
    Vector3d d_t_c;
      //d_t_c.setZero(3);
      d_t_c(0) = d_H_c(0,3);
      d_t_c(1) = d_H_c(1,3);
      d_t_c(2) = d_H_c(2,3);

      // extracrion of the rotation
      yarp::sig::Matrix d_R_c;
      d_R_c.resize(3,3);

      for (int row=0; row<3; row++)
        {
         for (int col=0; col<3; col++)
           {
         d_R_c(row,col) = d_H_c(row,col);
           }
        }
      yarp::sig::Vector d_AxisAngle_c;
      d_AxisAngle_c = yarp::math::dcm2axis(d_R_c);

        // grouping translation and rotation errors

         d_eta_c.resize(6);

         d_eta_c.segment(0,3) = d_t_c;
         d_eta_c(3) = d_AxisAngle_c(0) * d_AxisAngle_c(3);
         d_eta_c(4) = d_AxisAngle_c(1) * d_AxisAngle_c(3);
         d_eta_c(5) = d_AxisAngle_c(2) * d_AxisAngle_c(3);
         
    return d_eta_c;
}

// Velocity Twist Matrix associated to the robot base
MatrixXd ReferencesCompensator::getBaseVelocityTwist_left(iCub::iKin::iKinChain *LeftLegChain)
{
    
    yarp::sig::Vector LeftFootPoseInBase;

    LeftFootPoseInBase = LeftLegChain->EndEffPose();

    yarp::sig::Matrix Rot_FootInBase;

    Rot_FootInBase = yarp::math::axis2dcm(LeftFootPoseInBase.subVector(3,6));

    MatrixXd b_R_sf, sf_R_b;   
    b_R_sf.resize(3,3);
    b_R_sf.setZero(3,3);
    
    // extraction of data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
        {
         for (int col=0; col<3; col++)
           {
         b_R_sf(row,col) = Rot_FootInBase(row,col);
           }
        }

    sf_R_b = b_R_sf.transpose();

    // translation
    Vector3d b_Posi_ll, st_Posi_ll;
         
        b_Posi_ll(0) = LeftFootPoseInBase[0];
        b_Posi_ll(1) = LeftFootPoseInBase[1];
        b_Posi_ll(2) = LeftFootPoseInBase[2];

        //st_Posi_ll = b_R_sf.transpose() * b_Posi_ll;

    Matrix3d Skew_ll_b;
         Skew_ll_b.setZero(3,3);

        Skew_ll_b(0,1) = -b_Posi_ll(2);
        Skew_ll_b(0,2) =  b_Posi_ll(1);
        Skew_ll_b(1,0) =  b_Posi_ll(2);
        Skew_ll_b(1,2) = -b_Posi_ll(0);
        Skew_ll_b(2,0) = -b_Posi_ll(1);
        Skew_ll_b(2,1) =  b_Posi_ll(0);


    // Map between base angular velocity and time derivative of base orientation angles.
    // Twist transformation matrices
        
        BaseVelocityTwist_left.block(0,0, 3,3) = MatrixXd::Identity(3,3);
        BaseVelocityTwist_left.block(0,3, 3,3) = Skew_ll_b;
        BaseVelocityTwist_left.block(3,3, 3,3) = MatrixXd::Identity(3,3);

    return BaseVelocityTwist_left;


}

// Velocity Twist Matrix associated to the robot base
MatrixXd ReferencesCompensator::getBaseVelocityTwist_right(iCub::iKin::iKinChain *RightLegChain)
{

    yarp::sig::Vector RightFootPoseInBase;

    RightFootPoseInBase = RightLegChain->EndEffPose();

    yarp::sig::Matrix Rot_FootInBase;

    Rot_FootInBase = yarp::math::axis2dcm(RightFootPoseInBase.subVector(3,6));

    MatrixXd b_R_sf, sf_R_b;
    b_R_sf.resize(3,3);
    b_R_sf.setZero(3,3);
    // extraction of data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
        {
         for (int col=0; col<3; col++)
           {
         b_R_sf(row,col) = Rot_FootInBase(row,col);
           }
        }



    sf_R_b = b_R_sf.transpose();

    // translation
    Vector3d b_Posi_rl, st_Posi_rl;
         
        b_Posi_rl(0) = RightFootPoseInBase[0];
        b_Posi_rl(1) = RightFootPoseInBase[1];
        b_Posi_rl(2) = RightFootPoseInBase[2];

        //st_Posi_ll = b_R_sf.transpose() * b_Posi_ll;

    Matrix3d Skew_rl_b;
        Skew_rl_b.setZero(3,3);

        Skew_rl_b(0,1) = -b_Posi_rl(2);
        Skew_rl_b(0,2) =  b_Posi_rl(1);
        Skew_rl_b(1,0) =  b_Posi_rl(2);
        Skew_rl_b(1,2) = -b_Posi_rl(0);
        Skew_rl_b(2,0) = -b_Posi_rl(1);
        Skew_rl_b(2,1) =  b_Posi_rl(0);


    // Map between base angular velocity and time derivative of base orientation angles.
    // Twist transformation matrices
   
    BaseVelocityTwist_right.block(0,0, 3,3) = MatrixXd::Identity(3,3);
    BaseVelocityTwist_right.block(0,3, 3,3) = Skew_rl_b;
    BaseVelocityTwist_right.block(3,3, 3,3) = MatrixXd::Identity(3,3);

    return BaseVelocityTwist_right;

   
}

// Compensation Task Jacobian of the left leg in the base
MatrixXd ReferencesCompensator::getCompTaskJacobianLeftLeg(iCub::iKin::iKinChain *LeftLegChain)
{

    yarp::sig::Matrix Jac_left_leg_yarp;

    Jac_left_leg_yarp = LeftLegChain->GeoJacobian();

    MatrixXd Jac_left_leg_eigen;

    Jac_left_leg_eigen.resize(Jac_left_leg_yarp.rows(), Jac_left_leg_yarp.cols());


    for (int row=0; row<Jac_left_leg_yarp.rows(); row++)
    {
        for(int col=0; col<Jac_left_leg_yarp.cols(); col++)
        {
            Jac_left_leg_eigen(row, col) = Jac_left_leg_yarp(row, col);
        }
    }

    // Matrix of the task jacobian

    CompTaskJacobianLeftLeg  = - ReferencesCompensator::getBaseVelocityTwist_left(LeftLegChain) * Jac_left_leg_eigen;

    return CompTaskJacobianLeftLeg;
}

// Compensated Task Jacobian of the right leg in the base
MatrixXd ReferencesCompensator::getCompTaskJacobianRightLeg(iCub::iKin::iKinChain *RightLegChain)
{

    //
    yarp::sig::Matrix Jac_right_leg_yarp;
    Jac_right_leg_yarp = RightLegChain->GeoJacobian();
    MatrixXd Jac_right_leg_eigen;
    Jac_right_leg_eigen.resize(Jac_right_leg_yarp.rows(), Jac_right_leg_yarp.cols());

    for (int row=0; row<Jac_right_leg_yarp.rows(); row++)
    {
        for(int col=0; col<Jac_right_leg_yarp.cols(); col++)
        {
            Jac_right_leg_eigen(row, col) = Jac_right_leg_yarp(row, col);
        }
    }

    // Matrix of the task jacobian

    CompTaskJacobianRightLeg = - ReferencesCompensator::getBaseVelocityTwist_right(RightLegChain) * Jac_right_leg_eigen;

    return CompTaskJacobianRightLeg;
}

// Compensated Task Jacobian of the stance leg in the base
MatrixXd ReferencesCompensator::getCompTaskJacobianStanceLeg(iCub::iKin::iKinChain *LeftLegChain, iCub::iKin::iKinChain *RightLegChain, bool left_stance)
{
    // Matrix of the task jacobian

    if (left_stance) // if the stnace foot is the left foot
    {
        CompTaskJacobianStanceLeg = ReferencesCompensator::getCompTaskJacobianLeftLeg(LeftLegChain);
    }
    else
    {
        CompTaskJacobianStanceLeg = ReferencesCompensator::getCompTaskJacobianRightLeg(RightLegChain);
    }

    return CompTaskJacobianStanceLeg;
}


// set the gain for the compensation
void ReferencesCompensator::setservoingGain(VectorXd sGain)
{
    ServoingGain = sGain;
}

// compute compensated left leg joints
VectorXd ReferencesCompensator::getCompensated_lljoints(MatrixXd d_H_c, iCub::iKin::iKinChain *LeftLegChain, double SamplingTime)
{
    /* compute compensated left joint as :
     q = integral(q_dot); 
     with :
        q_dot = - pinv(TaskJacobian)* sGain * feature_error */

    //VectorXd q_left, q_dot_left;
    MatrixXd gainMx = ServoingGain.asDiagonal();

    MatrixXd TaskJacobianLeftLeg;

    TaskJacobianLeftLeg = ReferencesCompensator::getJacobianForAxisAngle(d_H_c) * ReferencesCompensator::getCompTaskJacobianLeftLeg(LeftLegChain);

    MatrixPseudoInverse MxPsdInv;

    q_dot_left = - MxPsdInv.pseudoInverse(TaskJacobianLeftLeg) * ServoingGain.asDiagonal() * ReferencesCompensator::FeatureError_d_H_c(d_H_c);

    // Euler integration
    VectorXd y_t = SamplingTime * (q_dot_left_init  + q_dot_left)/2.;
    q_dot_left_init = q_dot_left;

    return y_t;

}

// compute compensated right leg joints
VectorXd ReferencesCompensator::getCompensated_rljoints(MatrixXd d_H_c, iCub::iKin::iKinChain *RightLegChain, double SamplingTime)
{
    /* compute compensated right joint as :
     q = integral(q_dot); 
     with :
        q_dot = - pinv(TaskJacobian)* sGain * feature_error */

    //VectorXd q_right, q_dot_right;
    MatrixXd gainMx = ServoingGain.asDiagonal();
    MatrixPseudoInverse MxPsdInv;

    MatrixXd TaskJacobianRightLeg;

    TaskJacobianRightLeg = ReferencesCompensator::getJacobianForAxisAngle(d_H_c) * ReferencesCompensator::getCompTaskJacobianRightLeg(RightLegChain);

    q_dot_right = - MxPsdInv.pseudoInverse(TaskJacobianRightLeg) * ServoingGain.asDiagonal() * ReferencesCompensator::FeatureError_d_H_c(d_H_c);

    // Euler integration
    VectorXd y_t = SamplingTime* (q_dot_right_init  + q_dot_right)/2.;
    q_dot_right_init = q_dot_right;

    return y_t;

}


// Compute the error of the CoM
void ReferencesCompensator::ComputeComError(  Discrete_CP_LIP_Model *DMod,
                                              CpStanceFootPose *CoPref, 
                                              Vector left_Leg_Pose,
                                              Vector right_Leg_Pose,
                                              VectorXd m_CoM_leftF,
                                              VectorXd CoM_measurements_offset, 
                                              VectorXd m_CoM_velocity, int CpModelOption, bool left_stance,
                                              VectorXd l_foot_FT_vector,
                                              VectorXd r_foot_FT_vector)
{
    /* This function computes the error of the CoM with respect to the stance foot.
        it uses the CoM positions measured with respect to the right stance foot as one
        of its inputs */

    Model_statesX_error.resize(DMod->StatesX.rows());
    Model_statesY_error.resize(DMod->StatesY.rows());

    Model_statesR_error.resize(DMod->StatesR.rows());

    // Homogeneous transformations
    MatrixXd T_rightFootInBase(4,4),
             T_leftFootInBase(4,4);
    
    T_leftFootInBase.setZero(4,4);
    T_leftFootInBase(3,3) = 1.;

    T_rightFootInBase.setZero(4,4);
    T_rightFootInBase(3,3) = 1.;

    // Eigen rotation matrix of the feet frames wrt. the base
    MatrixXd    Rot_rightFootInBase(3,3), 
                Rot_leftFootInBase(3,3);

    Rot_leftFootInBase.setZero(3,3);
    Rot_rightFootInBase.setZero(3,3);

    // yarp rotation matrix of the feet frames wrt. the base
    yarp::sig::Matrix   Rot_lF_Base, 
                        Rot_rF_Base;

    Rot_lF_Base = yarp::math::axis2dcm(left_Leg_Pose.subVector(3,6));
    Rot_rF_Base = yarp::math::axis2dcm(right_Leg_Pose.subVector(3,6));

    // form yarp matrix to Eigen Matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
           Rot_leftFootInBase(row, col)  = Rot_lF_Base(row, col);
           Rot_rightFootInBase(row, col) = Rot_rF_Base(row, col);
        }
    }

        // Transformation from icub foot frame to the Genaral foot frame
        MatrixXd T_icubF_GF, R_icubF_GF, invT_icubF_GF;
        T_icubF_GF.resize(4,4);
        T_icubF_GF.setZero(4,4);   T_icubF_GF(3,3) = 1.;

        R_icubF_GF.resize(3,3);
        R_icubF_GF.setZero(3,3);
        R_icubF_GF(0,2) =  1.0;
        R_icubF_GF(1,1) = -1.0;
        R_icubF_GF(2,0) =  1.0;

        //R_icubF_GF.setIdentity(3,3);

        T_icubF_GF.block(0,0, 3, 3) = R_icubF_GF;

            // From General base to  icub
        MatrixXd T_GB_icubB, R_GB_icubB, invT_GB_icubB;
        T_GB_icubB.resize(4,4);
        T_GB_icubB.setZero(4,4);   T_GB_icubB(3,3) = 1.;

        R_GB_icubB.resize(3,3);
        R_GB_icubB.setZero(3,3);
        R_GB_icubB(0,0) = -1.0;
        R_GB_icubB(1,1) = -1.0;
        R_GB_icubB(2,2) =  1.0;

        T_GB_icubB.block(0,0, 3, 3) = R_GB_icubB;

        // inverse transformation From icub Base to the General Base
        invT_GB_icubB.resize(4,4);
        invT_GB_icubB.setZero(4,4);   invT_GB_icubB(3,3) = 1.;

        invT_GB_icubB.block(0,0,3,3) = R_GB_icubB.transpose();


    // assigning translation and rotation to the Homogeneous transformation and multiplying by the
    // transformation from the foot support frame to the inertial frame.
    T_leftFootInBase(0,3) = left_Leg_Pose[0];
    T_leftFootInBase(1,3) = left_Leg_Pose[1];
    T_leftFootInBase(2,3) = left_Leg_Pose[2];
    T_leftFootInBase.block(0,0, 3,3) = Rot_leftFootInBase * R_icubF_GF.transpose();

    // Transf of left foot wrt global base frame
    T_leftFootInBase = invT_GB_icubB * T_leftFootInBase;

    T_rightFootInBase(0,3) = right_Leg_Pose[0];
    T_rightFootInBase(1,3) = right_Leg_Pose[1];
    T_rightFootInBase(2,3) = right_Leg_Pose[2];
    T_rightFootInBase.block(0,0, 3,3) = Rot_rightFootInBase * R_icubF_GF.transpose();

    // Transf of right foot wrt global base frame
    T_rightFootInBase = invT_GB_icubB * T_rightFootInBase;


    // Comptutation of the CoM wrt to the Global base frame
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    trsl_com_GBase = T_leftFootInBase.block(0,3, 3,1) + T_leftFootInBase.block(0,0, 3,3)* m_CoM_leftF; // + T_leftFootInBase.block(0,3, 3,1);



    // MatrixXd T_BaseInLeftFoot;

    // T_BaseInLeftFoot.resize(4,4); 
    // T_BaseInLeftFoot.setZero(4,4);
    // T_BaseInLeftFoot(3,3) = 1.;

    // T_BaseInLeftFoot.block(0,0, 3,3) = Rot_leftFootInBase.transpose();
    // T_BaseInLeftFoot.block(0,3, 3,1) =-Rot_leftFootInBase.transpose() * T_leftFootInBase.block(0,3, 3,1); 

    MatrixXd T_RightFootInLeftFoot(4,4);
 
    T_RightFootInLeftFoot.setZero(4,4);
    T_RightFootInLeftFoot(3,3) = 1.;

    // T_RightFootInLeftFoot.block(0,0, 3,3) = Rot_leftFootInBase.transpose() * Rot_rightFootInBase;
    // T_RightFootInLeftFoot.block(0,3, 3,1) = Rot_leftFootInBase.transpose() * T_rightFootInBase.block(0,3, 3,1)
    //                                        -Rot_leftFootInBase.transpose() * T_leftFootInBase.block(0,3, 3,1); 

    T_RightFootInLeftFoot = T_leftFootInBase.inverse() * T_rightFootInBase;

    MatrixXd T_LeftFootInRightFoot(4,4);
 
    T_LeftFootInRightFoot.setZero(4,4);
    T_LeftFootInRightFoot(3,3) = 1.;

    // T_LeftFootInRightFoot.block(0,0, 3,3) = Rot_rightFootInBase.transpose() * Rot_leftFootInBase;
    // T_LeftFootInRightFoot.block(0,3, 3,1) = Rot_rightFootInBase.transpose() * T_leftFootInBase.block(0,3, 3,1)
    //                                        -Rot_rightFootInBase.transpose() * T_rightFootInBase.block(0,3, 3,1); 

    T_LeftFootInRightFoot = T_rightFootInBase.inverse() * T_leftFootInBase;

    // CoM position of the template model relative to the stance foot
    VectorXd trsl_com_sft_inertial(3),  // expressed in the inertial frame
             trsl_com_sft(3);           // expressed in the stance foot frame
   // expressed in the inertial frame
    trsl_com_sft_inertial(0) = (DMod->StatesX)(0) - CoPref->CoPRefX;
    trsl_com_sft_inertial(1) = (DMod->StatesY)(0) - CoPref->CoPRefY;
    trsl_com_sft_inertial(2) =  DMod->zc;

    // // CoM velocity of the template model relative to the stance foot
    // VectorXd V_com_sft_inertial(3),  // expressed in the inertial frame
    //          V_com_sft(3);          // expressed in the stance foot frame

    // Velocity of the CoM
    switch (CpModelOption)
    {
        case 0:
        {
            V_com_sft_inertial(0) = sqrt(9.80/DMod->zc)*((DMod->StatesX)(1) - DMod->StatesX(0));
            V_com_sft_inertial(1) = sqrt(9.80/DMod->zc)*((DMod->StatesY)(1) - DMod->StatesY(0));
            V_com_sft_inertial(2) = 0.;

            break;
        }

        case 1:
        {
            V_com_sft_inertial(0) = (DMod->StatesX)(1);
            V_com_sft_inertial(1) = (DMod->StatesY)(1);
            V_com_sft_inertial(2) = 0.;

            break;
        }

        case 2:
        {
            V_com_sft_inertial(0) = (DMod->StatesX)(1);
            V_com_sft_inertial(1) = (DMod->StatesY)(1);
            V_com_sft_inertial(2) = 0.;

            break;
        }
    }

   // rotation matrix of the stance foot frame wrt. the inertial frame (of model)
    MatrixXd R_sft_inertial_f(3,3);

    R_sft_inertial_f.setZero(3,3);
    R_sft_inertial_f(0,0) =  cos(CoPref->CoPRefR);
    R_sft_inertial_f(0,1) = -sin(CoPref->CoPRefR);
    R_sft_inertial_f(1,0) =  sin(CoPref->CoPRefR);
    R_sft_inertial_f(1,1) =  cos(CoPref->CoPRefR);
    R_sft_inertial_f(2,2) =  1.;

    // Position
    trsl_com_sft = R_sft_inertial_f.transpose() * trsl_com_sft_inertial;
    // Velocity
    V_com_sft = R_sft_inertial_f.transpose() * V_com_sft_inertial;
    // filtered velocity
    // V_com_sft = FilterVeloCoM_sft->getRK4Integral(V_com_sft);

    //
    VectorXd m_Velo_CoM_InBase(3);
    // from yarp to Eigen
    // m_Velo_CoM_InBase(0) = m_CoM_velocity(0);
    // m_Velo_CoM_InBase(1) = m_CoM_velocity(1);
    // m_Velo_CoM_InBase(2) = m_CoM_velocity(2); 
    m_Velo_CoM_InBase = m_CoM_velocity;   

    // position of the stance foot in the inertial frame (model)
    VectorXd    P_Stf_in_Inertial(3);

                P_Stf_in_Inertial(0) = CoPref->CoPRefX;
                P_Stf_in_Inertial(1) = CoPref->CoPRefY;
                P_Stf_in_Inertial(2) = 0.;

    // Positionm of the CoM relative to the Feet
        // m_CoM_right_Foot(0) = m_CoM_rightF(0) - 0.0* CoM_measurements_offset(0);
        // m_CoM_right_Foot(1) = m_CoM_rightF(1) + 0.0* CoM_measurements_offset(1);
        // m_CoM_right_Foot(2) = m_CoM_rightF(2);
        // // m_CoM_right_Foot = m_CoM_rightF;

        // m_CoM_left_Foot = T_RightFootInLeftFoot.block(0,0, 3,3)* m_CoM_right_Foot + T_RightFootInLeftFoot.block(0,3, 3,1);

        m_CoM_left_Foot(0) = m_CoM_leftF(0) - 1.0* CoM_measurements_offset(0);
        m_CoM_left_Foot(1) = m_CoM_leftF(1) + 0.0* CoM_measurements_offset(1);
        m_CoM_left_Foot(2) = m_CoM_leftF(2);
        // m_CoM_right_Foot = m_CoM_leftF;

        m_CoM_right_Foot = T_LeftFootInRightFoot.block(0,0, 3,3)* m_CoM_left_Foot + T_LeftFootInRightFoot.block(0,3, 3,1);

        VectorXd CoM_bias(3);
        CoM_bias.setZero(3);

        
    if (left_stance)
    {
        
        CoM_bias(0) =  0.00;
        CoM_bias(1) = 0.0049;
        // Compute the CoM error in the inertial from the error relative to the stance frame
        // Position
        CoM_position_error = m_CoM_left_Foot - CoM_bias; //R_sft_inertial_f * (T_leftFootInBase.block(0,0, 3,3)).transpose() * R_GB_icubB.transpose() *m_Velo_CoM_InBase; //(1.0* m_CoM_left_Foot - 1.0*trsl_com_sft);
        // Velocity
        CoM_velocity_error = R_sft_inertial_f * (1.*(T_leftFootInBase.block(0,0, 3,3)).transpose() * R_GB_icubB.transpose() * m_Velo_CoM_InBase);// - V_com_sft);
        
    }
    else
    {

        CoM_bias(0) = 0.000;
        CoM_bias(1) = -0.0041;
        // Compute the CoM error in the inertial from the error relative to the stance frame
        // Position
        CoM_position_error = m_CoM_right_Foot - CoM_bias; //R_sft_inertial_f * (T_rightFootInBase.block(0,0, 3,3)).transpose() * R_GB_icubB.transpose()* m_Velo_CoM_InBase; //(1.0* m_CoM_right_Foot - 1.0*trsl_com_sft);
        // Velocity
        CoM_velocity_error = R_sft_inertial_f * (1.*(T_rightFootInBase.block(0,0, 3,3)).transpose() * R_GB_icubB.transpose()* m_Velo_CoM_InBase);// - V_com_sft);

    }
    
    //CoM_position_error = T_LeftFootInRightFoot.block(0,3, 3,1); //R_sft_inertial_f * (1.0* m_CoM_right_Foot - 0.0*trsl_com_sft);
    //CoM_position_error = T_RightFootInLeftFoot.block(0,3, 3,1); //R_sft_inertial_f * (1.0* m_CoM_right_Foot - 0.0*trsl_com_sft);

    // the CoM error to be used in the compensation should be written in the inertial frame

        // Velocity of the CoM
    // switch (CpModelOption)
    // {
    //     case 0:
    //     {
    //         // States error (this is in the stance foot frame)
    //         Model_statesX_error(0) = CoM_position_error(0);
    //         Model_statesX_error(1) = CoM_position_error(0) + 1/sqrt(9.80/DMod->zc) * CoM_velocity_error(0);

    //         Model_statesY_error(0) = CoM_position_error(1);
    //         Model_statesY_error(1) = CoM_position_error(1) + 1/sqrt(9.80/DMod->zc) * CoM_velocity_error(1);

    //         break;
    //     }

    //     case 1:
    //     {
    //         // States error (this is in the stance foot frame)
    //         Model_statesX_error(0) = CoM_position_error(0);
    //         Model_statesX_error(1) = CoM_velocity_error(0);

    //         Model_statesY_error(0) = CoM_position_error(1);
    //         Model_statesY_error(1) = CoM_velocity_error(1);

    //         break;
    //     }

    //     case 2:
    //     {
    //         // States error (this is in the stance foot frame)
    //         Model_statesX_error(0) = CoM_position_error(0);
    //         Model_statesX_error(1) = CoM_velocity_error(0);
    //         Model_statesX_error(2) = 0.0;

    //         Model_statesY_error(0) = CoM_position_error(1);
    //         Model_statesY_error(1) = CoM_velocity_error(1);
    //         Model_statesY_error(2) = 0.0;

    //         break;
    //     }
    // }

// STATES OBSERVATIONS
// ===================

  // transformation from force torque sensor frame to world foot frame
  // Eigen::MatrixXd Rot_FTorque2GFoot(3,3);
  // Rot_FTorque2GFoot.setZero(3,3);

  // Rot_FTorque2GFoot(0,0) =  1.0;
  // Rot_FTorque2GFoot(1,1) = -1.0;
  // Rot_FTorque2GFoot(2,2) = -1.0;

  // Eigen::MatrixXd Rot_6x6FTorque2GFoot(6,6);
  // Rot_6x6FTorque2GFoot.setZero(6,6);

  // Rot_6x6FTorque2GFoot.block(0,0, 3,3) = Rot_FTorque2GFoot;
  // Rot_6x6FTorque2GFoot.block(3,3, 3,3) = Rot_FTorque2GFoot;

  Eigen::VectorXd lfoot_FT_WorldFoot(6);
  Eigen::VectorXd rfoot_FT_WorldFoot(6);

  // lfoot_FT_WorldFoot = Rot_6x6FTorque2GFoot * l_foot_FT_vector;
  // rfoot_FT_WorldFoot = Rot_6x6FTorque2GFoot * r_foot_FT_vector;

  lfoot_FT_WorldFoot = l_foot_FT_vector;
  rfoot_FT_WorldFoot = r_foot_FT_vector;

  // Computation of the ZMP with respect to the stance foot
    // the ZMP wrt to the foot origin is computed as 
    // x_ZMP = - tau_y/f_z
    // y_ZMP = tau_x/f_z
    // 
  //cout << " T_LeftFootInRightFoot : \n"<< T_LeftFootInRightFoot << endl;
  //cout << " T_RightFootInLeftFoot : \n"<< T_RightFootInLeftFoot << endl;


  Eigen::VectorXd lfoot_FT_WorldFoot_rFoot(6);
  Eigen::VectorXd rfoot_FT_WorldFoot_lFoot(6);

      // compute the right force/torque measurement wrt. the left stance
    rfoot_FT_WorldFoot_lFoot.segment(0, 3) = T_RightFootInLeftFoot.block(0,0, 3,3) * rfoot_FT_WorldFoot.segment(0, 3);
    rfoot_FT_WorldFoot_lFoot.segment(3, 3) = T_RightFootInLeftFoot.block(0,0, 3,3) * rfoot_FT_WorldFoot.segment(3, 3);

        // compute the right force/torque measurement wrt. the left stance
    lfoot_FT_WorldFoot_rFoot.segment(0, 3) = T_LeftFootInRightFoot.block(0,0, 3,3) * lfoot_FT_WorldFoot.segment(0, 3);
    lfoot_FT_WorldFoot_rFoot.segment(3, 3) = T_LeftFootInRightFoot.block(0,0, 3,3) * lfoot_FT_WorldFoot.segment(3, 3);

    double WlkMode;
    if (true)
    {
      WlkMode = -1.;
    }
    else
    {
      WlkMode = 1.;
    }
 

  if (left_stance)
  {
    // the ZMP wrt to the foot origin
    m_xZMP  = WlkMode *(-(lfoot_FT_WorldFoot(4) + rfoot_FT_WorldFoot_lFoot(4) - 0.0)/(lfoot_FT_WorldFoot(2)+rfoot_FT_WorldFoot_lFoot(2))-0.00) + 1.*CoPref->CoPRefX ;
    m_yZMP  = WlkMode *(lfoot_FT_WorldFoot(3) + rfoot_FT_WorldFoot_lFoot(3) -1.0)/(lfoot_FT_WorldFoot(2)+rfoot_FT_WorldFoot_lFoot(2)) + CoPref->CoPRefY;
    m_TauZ  =          lfoot_FT_WorldFoot(5) + rfoot_FT_WorldFoot_lFoot(5);
    m_disFx = lfoot_FT_WorldFoot(0) + rfoot_FT_WorldFoot_lFoot(0) + 10.0;
    m_disFy = lfoot_FT_WorldFoot(1) + rfoot_FT_WorldFoot_lFoot(1) + 20.0;

    //m_xZMP = -1.*(l_foot_FT_vector(4) + r_foot_FT_vector(4))/(l_foot_FT_vector(2)+r_foot_FT_vector(2)) + 1.*CoPref->CoPRefX -0.03+ 0.*DMod->ZMP_X; //-0.03
    //m_yZMP = 0.*(l_foot_FT_vector(3) + r_foot_FT_vector(3))/(l_foot_FT_vector(2)+r_foot_FT_vector(2));// + CoPref->CoPRefY;
    //m_TauZ = l_foot_FT_vector(5) + r_foot_FT_vector(5);
  }
  else
  {
    // the ZMP wrt to the foot origin
    m_xZMP = WlkMode *(-(lfoot_FT_WorldFoot_rFoot(4) + rfoot_FT_WorldFoot(4) - 0.0)/(lfoot_FT_WorldFoot_rFoot(2)+rfoot_FT_WorldFoot(2)) -0.00) + 1.*CoPref->CoPRefX;
    m_yZMP = WlkMode *(lfoot_FT_WorldFoot_rFoot(3) + rfoot_FT_WorldFoot(3) - 1.0)/(lfoot_FT_WorldFoot_rFoot(2)+rfoot_FT_WorldFoot(2)) + CoPref->CoPRefY;
    m_TauZ =         lfoot_FT_WorldFoot_rFoot(5) + rfoot_FT_WorldFoot(5);
    m_disFx = lfoot_FT_WorldFoot_rFoot(0) + rfoot_FT_WorldFoot(0) + 10.0;
    m_disFy = lfoot_FT_WorldFoot_rFoot(1) + rfoot_FT_WorldFoot(1) + 20.0;

    //m_xZMP = -1.*(l_foot_FT_vector(4) + r_foot_FT_vector(4))/(l_foot_FT_vector(2)+r_foot_FT_vector(2)) + 1.*CoPref->CoPRefX -0.03+ 0.*DMod->ZMP_X;
    //m_yZMP = 0.*(l_foot_FT_vector(3) + r_foot_FT_vector(3))/(l_foot_FT_vector(2)+r_foot_FT_vector(2));// + CoPref->CoPRefY;
    //m_TauZ = l_foot_FT_vector(5) + r_foot_FT_vector(5);
  }

//m_xZMP = CoPref->CoPRefX; //DMod->ZMP_X;
  // // Update of the Observer states
  // StatesX_Obs = A_Obs * StatesX_Obs + B_Obs * m_xZMP;
  // StatesY_Obs = A_Obs * StatesY_Obs + B_Obs * m_yZMP;

  VectorXd Abs_CoM(3), Abs_VeloCoM(3), Filtered_CoM(3);

    Abs_CoM(0) = CoM_position_error(0) * cos(CoPref->CoPRefR) - CoM_position_error(1) * sin(CoPref->CoPRefR) + CoPref->CoPRefX;
    Abs_CoM(1) = CoM_position_error(0) * sin(CoPref->CoPRefR) + CoM_position_error(1) * cos(CoPref->CoPRefR) + CoPref->CoPRefY;
    Abs_CoM(2) = 0.0;

    Filtered_CoM = FilterVeloCoM_sft->getRK4Integral(Abs_CoM);

    Abs_VeloCoM = (Abs_CoM - Filtered_CoM + CoM_velocity_error)/2.;
    Abs_VeloCoM = FilterVeloCoM_sft2->getRK4Integral(Abs_VeloCoM);


    // compute the error on the ZMP

    Delta_xZmp = m_xZMP - DMod->ZMP_X;
    Delta_yZmp = m_yZMP - DMod->ZMP_Y;


  switch (CpModelOption)
      {
          case 0:
          {
              VectorXd Ones_Obs(DMod->MxA.rows());
              Ones_Obs.setOnes(DMod->MxA.rows()); 
              //StatesX_Obs = A_Obs * StatesX_Obs + B_Obs * m_xZMP;
              //StatesX_Obs = DMod->MxA * StatesX_Obs + DMod->VeB * 1.0*m_xZMP + ObsGain * (DMod->StatesX(0)- Abs_CoM(0));  //working
             
              //StatesX_Obs = DMod->MxA * StatesX_Obs + DMod->VeB * DMod->ZMP_X; //(DMod->ZMP_X - CoPref->CoPRefX) + ObsGain * (DMod->StatesX(1)-CoPref->CoPRefX - StatesX_Obs(1));
              //StatesY_Obs = DMod->MxA * StatesX_Obs + DMod->VeB * (DMod->ZMP_Y - 0.*CoPref->CoPRefY) + ObsGain * (DMod->StatesY(1)-0.*CoPref->CoPRefY - StatesY_Obs(1));
              //StatesY_Obs = DMod->MxA * StatesY_Obs + DMod->VeB * (DMod->ZMP_Y-CoPref->CoPRefY) + ObsGain * (DMod->StatesY(1)-CoPref->CoPRefY - StatesY_Obs(1));
              DeltaStanceY = (PrevStanceY-CoPref->CoPRefY);
              PrevStanceY  = CoPref->CoPRefY;

              //StatesY_Obs = DMod->MxA * (StatesY_Obs+ Ones_Obs*DeltaStanceY) + DMod->VeB * (DMod->ZMP_Y-CoPref->CoPRefY) + ObsGain * (DMod->StatesY(1)-CoPref->CoPRefY - StatesY_Obs(1));
              //StatesY_Obs = DMod->MxA * (StatesY_Obs+ Ones_Obs*DeltaStanceY) + DMod->VeB * (m_yZMP-CoPref->CoPRefY)+ ObsGain * (DMod->StatesY(1)-CoPref->CoPRefY - StatesY_Obs(1));
              
              //StatesY_Obs = DMod->MxA * (StatesY_Obs) + DMod->VeB * (m_yZMP-DMod->ZMP_Y)- ObsGain * (StatesY_Obs(1)); // working active
              
              //StatesY_Obs = DMod->MxA * StatesY_Obs + DMod->VeB * m_yZMP + DMod->B_Dist * m_disFy + ObsGain * (DMod->StatesY(1) - StatesY_Obs(1));
              //StatesY_Obs = A_Obs * StatesY_Obs + B_Obs * (DMod->ZMP_Y - CoPref->CoPRefY); //m_yZMP + ObsGain * (DMod->StatesY(1) - StatesY_Obs(1));

              // Three states observer with ZMP
                A_Obs3.resize(3,3);
                A_Obs3.setZero(3,3);

                A_Obs3(0,0) = DMod->b1;
                A_Obs3(0,1) = DMod->a1 *(1.-pow(DMod->b1, 2.))/2.;
                A_Obs3(0,2) = 1.- DMod->a1*(1.+pow(DMod->b1, 2.))/2.;
                A_Obs3(1,1) = DMod->a1;
                A_Obs3(1,2) = 1.- DMod->a1;
                A_Obs3(2,2) = 1.;

                C_Obs3.resize(3);
                C_Obs3(0) = 0.0;
                C_Obs3(1) = 0.0;
                C_Obs3(2) = 1.0;

                //StatesY_Obs3 = (A_Obs3 - GainObs_3 * C_Obs3) * StatesY_Obs3 + GainObs_3 * (m_yZMP - DMod->ZMP_Y);
                StatesY_Obs3 = DMod->MxA_R * StatesY_Obs3 + GainObs_3 * (DMod->ZMP_Y - m_yZMP);

                Eigen::MatrixXd Com_Cp(2,2);
                Com_Cp(0,0) = 1.0;
                Com_Cp(0,1) = 0.0;
                Com_Cp(1,0) = 1.0;
                Com_Cp(1,1) = 1./DMod->W; 

                //StatesX_Obs = 1.* Com_Cp * X_motion.getRK4Solution(-DMod->Mass * DMod->W* DMod->W * (m_xZMP - DMod->ZMP_X)); //(-m_disFx/DMod->Mass);
                //StatesY_Obs = 1.* Com_Cp * Y_motion.getRK4Solution(-DMod->Mass * DMod->W* DMod->W * (m_yZMP - DMod->ZMP_Y)); //(-m_disFy/DMod->Mass);
                
                // This works well with feedback on the input velocity
                StatesX_Obs = X_motion.getRK4Solution(-DMod->Mass * DMod->W* DMod->W * (m_xZMP - DMod->ZMP_X)*0.126 * 0.0 + (-m_disFx - 22.0)); //0.126
                StatesY_Obs = Y_motion.getRK4Solution(-DMod->Mass * DMod->W* DMod->W * (m_yZMP - DMod->ZMP_Y)*0.25*0.0 + (-m_disFy +35.0)); // 0.15

                //StatesX_Obs = 1.* Com_Cp * X_motion.getRK4Solution(-DMod->Mass * DMod->W* DMod->W * (m_xZMP - DMod->ZMP_X)*0.2);  //not very effective
                //StatesY_Obs = 1.* Com_Cp * Y_motion.getRK4Solution(-DMod->Mass * DMod->W* DMod->W * (m_yZMP - DMod->ZMP_Y)*0.2);
                

              // States error (this is in the stance foot frame)
              //Model_statesX_error = StatesX_Obs;// - DMod->StatesX; //(0);//+CoPref->CoPRefY; // - DMod->StatesX;  // working: StatesX_Obs - DMod->StatesX;      
              
              Model_statesX_error(0) = StatesX_Obs(0);//Abs_CoM(0); //
              Model_statesX_error(1) = StatesX_Obs(1);//Abs_VeloCoM(0); //Abs_CoM(0) + 1./DMod->W*Abs_VeloCoM(0); //StatesX_Obs(1);//
              //Model_statesX_error(1) = StatesX_Obs(1);//+CoPref->CoPRefY; //DMod->ZMP_Y; //0.*StatesY_Obs(1); // - DMod->StatesX;     
              Model_statesY_error(0) = StatesY_Obs(0); //DMod->W* DMod->W * m_yZMP; //  - DMod->StatesY; //DMod->B_Dist * m_disFy;  //StatesY_Obs;// 
              Model_statesY_error(1) = StatesY_Obs(1); // StatesY_Obs; // - DMod->StatesY; //DMod->B_Dist * m_disFy;  //StatesY_Obs;// 
              // Model_statesX_error(0) = m_yZMP;   // - DMod->StatesX;
              // Model_statesX_error(1) = m_TauZ;     

              // Model_statesR_error(0) = 0.0;
              // Model_statesR_error(1) = 0.0;
              // Model_statesR_error(2) = -K_Wdot * m_TauZ;
              Model_statesR_error.segment(0,2) = R_motion.getRK4Solution(-K_Wdot * m_TauZ * 2.0); //1.4
              Model_statesR_error(2) = -0.*Model_statesR_error(0) - 10.0 * Model_statesR_error(1) - 1.*K_Wdot * m_TauZ;
              
              break;
          }

          case 1:
          {
              
              StatesX_Obs = A_Obs * StatesX_Obs + B_Obs * m_xZMP;
              StatesY_Obs = A_Obs * StatesY_Obs + B_Obs * m_yZMP;

              // States error (this is in the stance foot frame)
              Model_statesX_error = StatesX_Obs - DMod->StatesX;            
              Model_statesY_error = StatesY_Obs - DMod->StatesY;

              Model_statesR_error(0) = 0.0;
              Model_statesR_error(1) = 0.0;
              Model_statesR_error(2) = -K_Wdot * m_TauZ;

              break;
          }

          case 2:
          {
              // Derivation of the states from a two states model
                A_Obs.resize(2,2);
                A_Obs(0,0) = DMod->a1 *(1.+pow(DMod->b1, 2.))/2.;
                A_Obs(0,1) = DMod->a1/DMod->W *(1.-pow(DMod->b1, 2.))/2.;
                A_Obs(1,0) = DMod->a1*DMod->W *(1.-pow(DMod->b1, 2.))/2.;
                A_Obs(1,1) = DMod->a1 *(1.+pow(DMod->b1, 2.))/2.;

                // State Control vector
                B_Obs.resize(2);
                B_Obs(0) = 1.-DMod->a1*(1.+ pow(DMod->b1, 2.))/2.;
                B_Obs(1) = -DMod->a1*DMod->W *(1.-pow(DMod->b1, 2.))/2.;


                // update the states
                StatesX_Obs = A_Obs * StatesX_Obs + B_Obs * m_xZMP;
                StatesY_Obs = A_Obs * StatesY_Obs + B_Obs * m_yZMP;

              // 
              Model_statesX_error(0) = 0.0; //StatesX_Obs(0) - DMod->StatesX(0);
              Model_statesX_error(1) = 0.0; //StatesX_Obs(1) - DMod->StatesX(1);
              Model_statesX_error(2) = 0.0;

              Model_statesY_error(0) = 0.0; //StatesY_Obs(0) - DMod->StatesY(0);
              Model_statesY_error(1) = 0.0; //StatesY_Obs(1) - DMod->StatesY(1);
              Model_statesY_error(2) = 0.0;

              Model_statesR_error(0) = 0.0;
              Model_statesR_error(1) = 0.0;
              Model_statesR_error(2) = -K_Wdot * m_TauZ;

              break;
          }
      }
    

}

VectorXd ReferencesCompensator::getCoM_in_GlobalBase()
{
    return trsl_com_GBase;
}


VectorXd ReferencesCompensator::getCoM_PositionError()
{
    return CoM_position_error;
}

VectorXd ReferencesCompensator::getCoM_VelocityError()
{
    return CoM_velocity_error;

}

VectorXd ReferencesCompensator::get_ModelStatesErrorX()
{
    return Model_statesX_error;
}

VectorXd ReferencesCompensator::get_ModelStatesErrorY()
{
    return Model_statesY_error;

}

VectorXd ReferencesCompensator::get_ModelStatesErrorR()
{
    return Model_statesR_error;

}

void ReferencesCompensator::InitializeObserver(Discrete_CP_LIP_Model *DMod)
{
    A_Obs = DMod->MxA;
    B_Obs = DMod->VeB;

    StatesX_Obs = DMod->StatesX;
    StatesY_Obs = DMod->StatesY;// - MatrixXd::Identity(DMod->MxA.rows(),1)*DMod->ZMP_Y;

}

//
VectorXd ReferencesCompensator::get_m_Acceleration_InBase (VectorXd m_acceleration, Vector m_orientation_rpy)
{
    
    yarp::sig::Matrix m_Rot_Body_in_Inertial;
    VectorXd    gravity_vector(3);

                gravity_vector(0) =  0.00;
                gravity_vector(1) =  0.00;
                gravity_vector(2) = -9.80;

    MatrixXd     m_Rot_Inertial_In_Body(3,3);

    // Rotation from robot body to inertial Frame
    m_Rot_Body_in_Inertial = yarp::math::rpy2dcm(CTRL_DEG2RAD * m_orientation_rpy);

    // From yarp to Eigen and transpose of the rotation matrix
    for (int row=0; row<3; row++)
    {
        for(int col=0; col<3; col++)
        {
            m_Rot_Inertial_In_Body(col,row) = m_Rot_Body_in_Inertial(row, col);
        }
    }

    //cout << " m_Rot_Inertial_In_Body is : \n" << m_Rot_Inertial_In_Body << endl;
    //
    m_acceleration_in_Body = m_acceleration + m_Rot_Inertial_In_Body * gravity_vector;

    return m_acceleration_in_Body;


}

VectorXd ReferencesCompensator::getEstimatedCoMVelocity(double T, VectorXd m_acceleration, Vector m_orientation_rpy)
{
     // Compute the CoM velocity from the measured acceleration
    VectorXd y_t, m_acceleration_Base;

            m_acceleration_Base = ReferencesCompensator::get_m_Acceleration_InBase (m_acceleration, m_orientation_rpy);

    est_velocity_base = T * (m_acceleration_init + m_acceleration_Base)/2.;

    //est_velocity_base = y_t;

    m_acceleration_init = m_acceleration_Base;

    return est_velocity_base;

}

double ReferencesCompensator::get_xZmpError()
{
   return Delta_xZmp;
}

double ReferencesCompensator::get_yZmpError()
{
   return Delta_yZmp;
}

