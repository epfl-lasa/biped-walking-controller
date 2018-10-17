

#include <cmath>
#include <iostream>

#include "Grasping.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


VectorXd axis_angle(MatrixXd d_R_c)
{

    double angle;
    angle = acos(0.5*(d_R_c.trace() - 1.0));

    VectorXd axis(3), temp(3);

    temp(0) = d_R_c(2,1) - d_R_c(1,2);
    temp(1) = d_R_c(0,2) - d_R_c(2,0);
    temp(2) = d_R_c(1,0) - d_R_c(0,1);

    axis = 0.5 * (1./sin(angle + 0.000001)) * temp;

    return angle * axis;
}

Grasping::Grasping(){}

Grasping::~Grasping() 
{
    if (qpOSolver_lhand) {
        delete qpOSolver_lhand;
        qpOSolver_lhand = 0;
    }

    if (qpOSolver_rhand) {
        delete qpOSolver_rhand;
        qpOSolver_rhand = 0;
    }

}
//
void Grasping::Inititialize(iCub::iKin::iKinChain *left_Arm_Chain,
                            iCub::iKin::iKinChain *right_Arm_Chain)
{
	// rotation matrix from iCub foot frame to the general foot frame
    Rot_icubFoot_GenFoot.resize(3,3);
    Rot_icubFoot_GenFoot.setZero(3,3);
    Rot_icubFoot_GenFoot(0,2) =  1.0;
    Rot_icubFoot_GenFoot(1,1) = -1.0;
    Rot_icubFoot_GenFoot(2,0) =  1.0;

    Rot6x6_root_Glstance.resize(6,6);
    Rot6x6_root_Glstance.setZero(6,6);

    Rot6x6_root_Grstance.resize(6,6);
    Rot6x6_root_Grstance.setZero(6,6);

    InteractionMx_Mu_Theta.resize(6,6);
    InteractionMx_Mu_Theta     = MatrixXd::Identity(6,6);

    RootVeloTwistMx_lhand_lleg = MatrixXd::Identity(6,6);
    RootVeloTwistMx_lhand_rleg = MatrixXd::Identity(6,6);
    RootVeloTwistMx_rhand_lleg = MatrixXd::Identity(6,6);
    RootVeloTwistMx_rhand_rleg = MatrixXd::Identity(6,6);

    Glstance_lhand_VeloTwistMx = MatrixXd::Identity(6,6);
    Glstance_rhand_VeloTwistMx = MatrixXd::Identity(6,6);
    Grstance_lhand_VeloTwistMx = MatrixXd::Identity(6,6);
    Grstance_rhand_VeloTwistMx = MatrixXd::Identity(6,6);

    Lhand_GenStanceLegVeloTwistMx = MatrixXd::Identity(6,6);
    Rhand_GenStanceLegVeloTwistMx = MatrixXd::Identity(6,6);

    Root_LeftStanceLeg_Jacobian.resize(6,6); 
    Root_LeftStanceLeg_Jacobian = MatrixXd::Identity(6,6);   
    Root_RightStanceLeg_Jacobian.resize(6,6);
    Root_RightStanceLeg_Jacobian = MatrixXd::Identity(6,6);

    graspGain_lh.resize(6);
    graspGain_rh.resize(6);

    for (int i=0; i<6; i++)
    {
        graspGain_lh(i) = 1.0;
        graspGain_rh(i) = 1.0;
    }

    World_lhand_VeloTwistMx.resize(6,6);
    World_lhand_VeloTwistMx.setIdentity(6,6);

    World_rhand_VeloTwistMx.resize(6,6);
    World_rhand_VeloTwistMx.setIdentity(6,6);

    Lhand_World_VeloTwistMx.resize(6,6);
    Lhand_World_VeloTwistMx.setIdentity(6,6);

    Rhand_World_VeloTwistMx.resize(6,6);
    Rhand_World_VeloTwistMx.setIdentity(6,6);

    RootVeloTwistMx_lh_World.resize(6,6);
    RootVeloTwistMx_lh_World.setIdentity(6,6);

    RootVeloTwistMx_rh_World.resize(6,6);
    RootVeloTwistMx_rh_World.setIdentity(6,6);

    Rot6x6_root_World.resize(6,6);
    Rot6x6_root_World.setIdentity(6,6);

    // Joint velocity of the graasping task
    qDot_Grasping_left_hand.resize(left_Arm_Chain->getDOF());
    qDot_Grasping_left_hand.setZero(left_Arm_Chain->getDOF());

    qDot_Grasping_right_hand.resize(right_Arm_Chain->getDOF());
    qDot_Grasping_right_hand.setZero(right_Arm_Chain->getDOF());

    qDot_Grasping_bimanual.resize(left_Arm_Chain->getDOF() + right_Arm_Chain->getDOF());
    qDot_Grasping_bimanual.setZero(left_Arm_Chain->getDOF() + right_Arm_Chain->getDOF());

    //
    desired_lhand_pose_in_root.resize(7);
    desired_rhand_pose_in_root.resize(7);

    if(false)
    {
        //
        desired_lhand_pose_in_root[0] = -0.276295;
        desired_lhand_pose_in_root[1] = -0.152659;
        desired_lhand_pose_in_root[2] =  0.018416;
        desired_lhand_pose_in_root[3] =  0.111949;
        desired_lhand_pose_in_root[4] =  0.615117;
        desired_lhand_pose_in_root[5] = -0.780447;
        desired_lhand_pose_in_root[6] =  2.955697;

        desired_rhand_pose_in_root[0] = -0.273784;
        desired_rhand_pose_in_root[1] =  0.136454;
        desired_rhand_pose_in_root[2] =  0.007928;
        desired_rhand_pose_in_root[3] =  0.075938;
        desired_rhand_pose_in_root[4] =  0.798382;
        desired_rhand_pose_in_root[5] = -0.597344;
        desired_rhand_pose_in_root[6] =  2.833639;
    }
    else
    {
        //
        desired_lhand_pose_in_root[0] = -0.300000;
        desired_lhand_pose_in_root[1] = -0.100000;
        desired_lhand_pose_in_root[2] =  0.060000;
        desired_lhand_pose_in_root[3] = -0.000000;
        desired_lhand_pose_in_root[4] =  0.707107;
        desired_lhand_pose_in_root[5] = -0.707107;
        desired_lhand_pose_in_root[6] =  3.141593;

        desired_rhand_pose_in_root[0] = -0.300000;
        desired_rhand_pose_in_root[1] =  0.100000;
        desired_rhand_pose_in_root[2] =  0.060000;
        desired_rhand_pose_in_root[3] =  0.000000;
        desired_rhand_pose_in_root[4] =  0.707107; // +
        desired_rhand_pose_in_root[5] = -0.707107; //-
        desired_rhand_pose_in_root[6] =  3.141593;
    }
    // //
    // Desired_Left_Arm_PoseAsAxisAngles.resize(7);
    // Desired_Left_Arm_PoseAsAxisAngles.setSubvector(0, desired_lhand_pose_in_root.subVector(0,2));
    // yarp::sig::Vector Des_Left_Arm_orientation_axis = (1./yarp::math::norm(desired_lhand_pose_in_root.subVector(3,5)))*desired_lhand_pose_in_root.subVector(3,5);
    // Desired_Left_Arm_PoseAsAxisAngles.setSubvector(3, Des_Left_Arm_orientation_axis);
    // Desired_Left_Arm_PoseAsAxisAngles[6] = desired_lhand_pose_in_root[6];

    // Desired_Right_Arm_PoseAsAxisAngles.resize(7);
    // Desired_Right_Arm_PoseAsAxisAngles.setSubvector(0, desired_rhand_pose_in_root.subVector(0,2));
    // yarp::sig::Vector Des_Right_Arm_orientation_axis = (1./yarp::math::norm(desired_rhand_pose_in_root.subVector(3,5)))*desired_rhand_pose_in_root.subVector(3,5);
    // Desired_Right_Arm_PoseAsAxisAngles.setSubvector(3, Des_Right_Arm_orientation_axis);
    // Desired_Right_Arm_PoseAsAxisAngles[6] = desired_rhand_pose_in_root[6];
    //
    yarp::sig::Vector Des_Left_Arm_orientation_axis = (1./yarp::math::norm(desired_lhand_pose_in_root.subVector(3,5)))*desired_lhand_pose_in_root.subVector(3,5);
    desired_lhand_pose_in_root.setSubvector(3, Des_Left_Arm_orientation_axis);
       
    yarp::sig::Vector Des_Right_Arm_orientation_axis = (1./yarp::math::norm(desired_rhand_pose_in_root.subVector(3,5)))*desired_rhand_pose_in_root.subVector(3,5);
    desired_rhand_pose_in_root.setSubvector(3, Des_Right_Arm_orientation_axis);
    //
    Desired_Left_Arm_PoseAsAxisAngles.resize(7);
    Desired_Right_Arm_PoseAsAxisAngles.resize(7);

    Desired_Left_Arm_PoseAsAxisAngles = desired_lhand_pose_in_root;
    Desired_Right_Arm_PoseAsAxisAngles = desired_rhand_pose_in_root;

    // Homogenaous transfromation of the current hand frame wrt to the desired one
    HTrsf_CurLh_2_DesLh.setIdentity();
    HTrsf_CurRh_2_DesRh.setIdentity();

    

    // initialize admittance parameters
    // -----------------------------------------
    
    // Properties of the admittance law
    lh_virtualInertia.resize(6);   lh_virtualInertia << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    rh_virtualInertia.resize(6);   rh_virtualInertia << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    lh_virtualDamping.resize(6);   lh_virtualDamping << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    rh_virtualDamping.resize(6);   rh_virtualDamping << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    lh_virtualStiffness.resize(6);   lh_virtualStiffness << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    rh_virtualStiffness.resize(6);   rh_virtualStiffness << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    //------------------------------
    int nlh_jts = left_Arm_Chain->getDOF();

    Weight_qDot.resize(nlh_jts);
    Weight_qDot.setOnes();
    Weight_qDot *= 0.0001;

    velocity_Saturation_Limit.resize(nlh_jts);
    velocity_Saturation_Limit.setOnes();
    velocity_Saturation_Limit *= M_PI/6.0;

    Gsp_Hessian_matrix_lhand.resize(nlh_jts, nlh_jts);
    Gsp_Hessian_matrix_lhand.setZero();
    Gsp_Gradient_vector_lhand.resize(nlh_jts);
    Gsp_Gradient_vector_lhand.setZero();
    Gsp_Hessian_matrix_rhand.resize(nlh_jts,nlh_jts);
    Gsp_Hessian_matrix_rhand.setZero();
    Gsp_Gradient_vector_rhand.resize(nlh_jts);
    Gsp_Gradient_vector_rhand.setZero();

    Inequality_matrix_lhand.resize(4*nlh_jts,nlh_jts);
    Inequality_matrix_lhand.setZero();
    Inequality_vector_lhand.resize(4*nlh_jts);
    Inequality_vector_lhand.setZero();
    Inequality_matrix_rhand.resize(4*nlh_jts,nlh_jts);
    Inequality_matrix_rhand.setZero();
    Inequality_vector_rhand.resize(4*nlh_jts);
    Inequality_vector_rhand.setZero();
    


}

// Vector of feature error
VectorXd Grasping::getPoseError_d_H_c(MatrixXd d_H_c)
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
    d_eta_c(3) = d_AxisAngle_c[0] * d_AxisAngle_c[3];
    d_eta_c(4) = d_AxisAngle_c[1] * d_AxisAngle_c[3];
    d_eta_c(5) = d_AxisAngle_c[2] * d_AxisAngle_c[3];

    // Vector3d Mu_theta;
    // Mu_theta = axis_angle(d_R_c);

    // d_eta_c(3) = Mu_theta(0);
    // d_eta_c(4) = Mu_theta(1);
    // d_eta_c(5) = Mu_theta(2);
             
    return d_eta_c;

}

// Vector of feature error
VectorXd Grasping::getPoseError_d_H_c2(MatrixXd d_H_c)
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

    MatrixXd c_H_d;
    c_H_d = d_H_c.inverse();


    d_eta_c.resize(6);
    d_eta_c.segment(0,3) = c_H_d.block(0,3,3,1);
    d_eta_c(3) = d_AxisAngle_c[0] * d_AxisAngle_c[3];
    d_eta_c(4) = d_AxisAngle_c[1] * d_AxisAngle_c[3];
    d_eta_c(5) = d_AxisAngle_c[2] * d_AxisAngle_c[3];

    // VectorXd Mu_theta;
    // Mu_theta = axis_angle(d_R_c);

    // d_eta_c(3) = Mu_theta(0);
    // d_eta_c(4) = Mu_theta(1);
    // d_eta_c(5) = Mu_theta(2);
             
    return d_eta_c;

}

MatrixXd Grasping::getInteractionMxForAxisAngle(MatrixXd d_H_c)
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

    // VectorXd Mu_theta;
    // Mu_theta = axis_angle(d_R_c);
    // double theta;
    // VectorXd Mu(3);
    // theta = sqrt(Mu_theta(0)*Mu_theta(0) + Mu_theta(1)*Mu_theta(1) + Mu_theta(2)*Mu_theta(2));
    // Mu    = 1./theta * Mu_theta;


    // function sinc(theta) and sinc(theta/2)
    double sinc_theta, sinc_theta_2;

    sinc_theta   = sin(d_AxisAngle_c[3] + 1e-6)/(d_AxisAngle_c[3] + 1e-6);
    sinc_theta_2 = sin((d_AxisAngle_c[3] + 1e-6)/2.)/((d_AxisAngle_c[3] + 1e-6)/2.);

        
    Matrix3d Skew_Mu;
    Skew_Mu.resize(3,3);
    Skew_Mu.setZero(3,3);

    Skew_Mu(0,1) = -d_AxisAngle_c[2];
    Skew_Mu(0,2) =  d_AxisAngle_c[1];
    Skew_Mu(1,0) =  d_AxisAngle_c[2];
    Skew_Mu(1,2) = -d_AxisAngle_c[0];
    Skew_Mu(2,0) = -d_AxisAngle_c[1];
    Skew_Mu(2,1) =  d_AxisAngle_c[0];

    // Jacobian of the rotation
    Matrix3d L_Mu_Theta;
    L_Mu_Theta.resize(3,3);
    L_Mu_Theta.setIdentity(3,3);

    L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c[3]/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

    // Building the overall jacobian

    InteractionMx_Mu_Theta.block(0,0, 3,3) = d_H_c.block(0,0, 3,3);
    InteractionMx_Mu_Theta.block(3,3, 3,3) = L_Mu_Theta;

    return InteractionMx_Mu_Theta;

}

MatrixXd Grasping::getInteractionMxForAxisAngle2(MatrixXd d_H_c)
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

    sinc_theta   = sin(d_AxisAngle_c[3] + 1e-6)/(d_AxisAngle_c[3] + 1e-6);
    sinc_theta_2 = sin((d_AxisAngle_c[3] + 1e-6)/2.)/((d_AxisAngle_c[3] + 1e-6)/2.);

        
    Matrix3d Skew_Mu;
    Skew_Mu.resize(3,3);
    Skew_Mu.setZero(3,3);

    Skew_Mu(0,1) = -d_AxisAngle_c[2];
    Skew_Mu(0,2) =  d_AxisAngle_c[1];
    Skew_Mu(1,0) =  d_AxisAngle_c[2];
    Skew_Mu(1,2) = -d_AxisAngle_c[0];
    Skew_Mu(2,0) = -d_AxisAngle_c[1];
    Skew_Mu(2,1) =  d_AxisAngle_c[0];


    MatrixXd c_H_d;
    c_H_d = d_H_c.inverse();

    Matrix3d Skew_c_t_d;
    Skew_c_t_d.resize(3,3);
    Skew_c_t_d.setZero(3,3);

    Skew_c_t_d(0,1) = -c_H_d(2,3);
    Skew_c_t_d(0,2) =  c_H_d(1,3);
    Skew_c_t_d(1,0) =  c_H_d(2,3);
    Skew_c_t_d(1,2) = -c_H_d(0,3);
    Skew_c_t_d(2,0) = -c_H_d(1,3);
    Skew_c_t_d(2,1) =  c_H_d(0,3);


    // Jacobian of the rotation
    Matrix3d L_Mu_Theta;
    L_Mu_Theta.resize(3,3);
    L_Mu_Theta.setIdentity(3,3);

    L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c[3]/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

    // Building the overall jacobian

    InteractionMx_Mu_Theta.block(0,0, 3,3) = -1.0*MatrixXd::Identity(3,3);
    InteractionMx_Mu_Theta.block(0,3, 3,3) = Skew_c_t_d;
    InteractionMx_Mu_Theta.block(3,3, 3,3) = L_Mu_Theta;

    return InteractionMx_Mu_Theta;

}



// root Jocabian matrix associated with left stance leg
MatrixXd Grasping::getRoot_LeftStanceLeg_Jacobian(iCub::iKin::iKinChain *LeftLegChain)
{
    
    // 
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_FootInBase;       // Rotation of the left foot frame wrt. the root frame

    LeftFootPoseInRoot = LeftLegChain->EndEffPose();
    Rot_FootInBase     = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));

    MatrixXd ll_R_rt,   // 
             Gll_R_rt;   
    ll_R_rt.resize(3,3);
    ll_R_rt.setZero(3,3);
    
    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
             ll_R_rt(row,col) = Rot_FootInBase(col,row);
        }
    }

    // Rotation Mx of the root frame wrt the a general left foot frame 
    Gll_R_rt = Rot_icubFoot_GenFoot * ll_R_rt;

    // translation 
    Vector3d rt_Posi_ll, Glstance_Posi_ll;
         
    rt_Posi_ll(0) = LeftFootPoseInRoot[0];
    rt_Posi_ll(1) = LeftFootPoseInRoot[1];
    rt_Posi_ll(2) = LeftFootPoseInRoot[2];

    Glstance_Posi_ll   = Gll_R_rt * rt_Posi_ll;

    Matrix3d Skew_ll_Glstance;
             Skew_ll_Glstance.setZero(3,3);

        Skew_ll_Glstance(0,1) = -Glstance_Posi_ll(2);
        Skew_ll_Glstance(0,2) =  Glstance_Posi_ll(1);
        Skew_ll_Glstance(1,0) =  Glstance_Posi_ll(2);
        Skew_ll_Glstance(1,2) = -Glstance_Posi_ll(0);
        Skew_ll_Glstance(2,0) = -Glstance_Posi_ll(1);
        Skew_ll_Glstance(2,1) =  Glstance_Posi_ll(0);


    // Map between base angular velocity and time derivative of base orientation angles.
    // Twist transformation matrices
        // Root_LeftStanceLeg_Jacobian.resize(6,6);
        // Root_LeftStanceLeg_Jacobian.setZero(6,6);
        
        Root_LeftStanceLeg_Jacobian.block(0,0, 3,3) = Gll_R_rt;
        Root_LeftStanceLeg_Jacobian.block(0,3, 3,3) = Skew_ll_Glstance * Gll_R_rt;
        Root_LeftStanceLeg_Jacobian.block(3,3, 3,3) = Gll_R_rt;

    return Root_LeftStanceLeg_Jacobian;  

}

// root Jocabian matrix associated with right stance leg
MatrixXd Grasping::getRoot_RightStanceLeg_Jacobian(iCub::iKin::iKinChain *RightLegChain)
{

    yarp::sig::Vector RightFootPoseInRoot;
    yarp::sig::Matrix Rot_FootInBase;

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_FootInBase      = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));

    MatrixXd rl_R_rt,       // 
             Grl_R_rt;   
    rl_R_rt.resize(3,3);
    rl_R_rt.setZero(3,3);
    
    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
             rl_R_rt(row,col) = Rot_FootInBase(col,row);
        }
    }

    // Rotation Mx of the root frame wrt the a general right foot frame 
    Grl_R_rt = Rot_icubFoot_GenFoot * rl_R_rt;

    // translation
    Vector3d rt_Posi_rl, Grstance_Posi_rl;
         
        rt_Posi_rl(0) = RightFootPoseInRoot[0];
        rt_Posi_rl(1) = RightFootPoseInRoot[1];
        rt_Posi_rl(2) = RightFootPoseInRoot[2];

        Grstance_Posi_rl   = Grl_R_rt * rt_Posi_rl;

    Matrix3d Skew_rl_Grstance;
             Skew_rl_Grstance.setZero(3,3);

        Skew_rl_Grstance(0,1) = -Grstance_Posi_rl(2);
        Skew_rl_Grstance(0,2) =  Grstance_Posi_rl(1);
        Skew_rl_Grstance(1,0) =  Grstance_Posi_rl(2);
        Skew_rl_Grstance(1,2) = -Grstance_Posi_rl(0);
        Skew_rl_Grstance(2,0) = -Grstance_Posi_rl(1);
        Skew_rl_Grstance(2,1) =  Grstance_Posi_rl(0);


    // Map between base angular velocity and time derivative of base orientation angles.
    // Twist transformation matrices

        // Root_RightStanceLeg_Jacobian.resize(6,6);
        // Root_RightStanceLeg_Jacobian.setZero(6,6);
        
        Root_RightStanceLeg_Jacobian.block(0,0, 3,3) = Grl_R_rt;
        Root_RightStanceLeg_Jacobian.block(0,3, 3,3) = Skew_rl_Grstance * Grl_R_rt;
        Root_RightStanceLeg_Jacobian.block(3,3, 3,3) = Grl_R_rt;

    return Root_RightStanceLeg_Jacobian;

}

//  h_W_w   w: gen stance foot
// ~~~~~~~
// Velocity twist matrix of a General stance foot frame (taken as the wolrd frame) wrt. the left hand frame (h_W_w)
MatrixXd Grasping::getLhand_GenStanceLegVeloTwistMx( iCub::iKin::iKinChain *left_Arm_Chain, 
                                                     iCub::iKin::iKinChain *LeftLegChain,
                                                     iCub::iKin::iKinChain *RightLegChain,
                                                     bool StanceLeg)
{
    yarp::sig::Vector LeftHandPoseInRoot;   // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    Eigen::MatrixXd   Trsf_LHandInRoot;     // Homogeneous transformation of the left hand frame wrt. the root frame
  
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    Eigen::MatrixXd   Trsf_LeftFootInRoot;  // Homogeneous transformation of the left leg frame wrt. the root frame
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame
    Eigen::MatrixXd   Trsf_RightFootInRoot; // Homogeneous transformation of the right leg frame wrt. the root frame

    

    LeftHandPoseInRoot  = left_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(LeftHandPoseInRoot.subVector(3,6));
   
    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));




    Trsf_LHandInRoot.resize(4,4);    Trsf_LHandInRoot.setZero(4,4);    Trsf_LHandInRoot(3,3)     = 1.0;
    Trsf_LHandInRoot(0,3) = LeftHandPoseInRoot[0];
    Trsf_LHandInRoot(1,3) = LeftHandPoseInRoot[1];
    Trsf_LHandInRoot(2,3) = LeftHandPoseInRoot[2];
    
    Trsf_LeftFootInRoot.resize(4,4);    Trsf_LeftFootInRoot.setZero(4,4);    Trsf_LeftFootInRoot(3,3)  = 1.0;
    Trsf_LeftFootInRoot(0,3) = LeftFootPoseInRoot[0];
    Trsf_LeftFootInRoot(1,3) = LeftFootPoseInRoot[1];
    Trsf_LeftFootInRoot(2,3) = LeftFootPoseInRoot[2];

    Trsf_RightFootInRoot.resize(4,4);   Trsf_RightFootInRoot.setZero(4,4);   Trsf_RightFootInRoot(3,3) = 1.0;
    Trsf_RightFootInRoot(0,3) = RightFootPoseInRoot[0];
    Trsf_RightFootInRoot(1,3) = RightFootPoseInRoot[1];
    Trsf_RightFootInRoot(2,3) = RightFootPoseInRoot[2];

    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
            Trsf_LHandInRoot(row,col)     = Rot_HandInRoot(row,col);
            Trsf_LeftFootInRoot(row,col)  = Rot_LeftFootInRoot(row,col);
            Trsf_RightFootInRoot(row,col) = Rot_RightFootInRoot(row,col);
        }
    }
   

    MatrixXd Trsf_GenFoot_icubFoot(4,4);
             Trsf_GenFoot_icubFoot.setZero(4,4);    Trsf_GenFoot_icubFoot(3,3) = 1.0;
             Trsf_GenFoot_icubFoot.block(0,0, 3,3) = Rot_icubFoot_GenFoot.transpose();


    MatrixXd Trsf_GenFoot_Lhand(4,4);

    if(StanceLeg)
    {
        // left hand with left leg as stance foot
        Trsf_GenFoot_Lhand = Trsf_LHandInRoot.inverse() * Trsf_LeftFootInRoot * Trsf_GenFoot_icubFoot;

        Matrix3d Skew_Glstance_lh;
                 Skew_Glstance_lh.setZero(3,3);

            Skew_Glstance_lh(0,1) = -Trsf_GenFoot_Lhand(2,3);
            Skew_Glstance_lh(0,2) =  Trsf_GenFoot_Lhand(1,3);
            Skew_Glstance_lh(1,0) =  Trsf_GenFoot_Lhand(2,3);
            Skew_Glstance_lh(1,2) = -Trsf_GenFoot_Lhand(0,3);
            Skew_Glstance_lh(2,0) = -Trsf_GenFoot_Lhand(1,3);
            Skew_Glstance_lh(2,1) =  Trsf_GenFoot_Lhand(0,3);

            Glstance_lhand_VeloTwistMx.block(0,0, 3,3) = Trsf_GenFoot_Lhand.block(0,0, 3,3);
            Glstance_lhand_VeloTwistMx.block(0,3, 3,3) = Skew_Glstance_lh * Trsf_GenFoot_Lhand.block(0,0, 3,3);
            Glstance_lhand_VeloTwistMx.block(3,3, 3,3) = Trsf_GenFoot_Lhand.block(0,0, 3,3);

            Lhand_GenStanceLegVeloTwistMx = Glstance_lhand_VeloTwistMx;

    }
    else
    {
        // left hand with right leg as stance foot   
        Trsf_GenFoot_Lhand = Trsf_LHandInRoot.inverse() * Trsf_RightFootInRoot * Trsf_GenFoot_icubFoot;

        Matrix3d Skew_Grstance_lh;
             Skew_Grstance_lh.setZero(3,3);

            Skew_Grstance_lh(0,1) = -Trsf_GenFoot_Lhand(2,3);
            Skew_Grstance_lh(0,2) =  Trsf_GenFoot_Lhand(1,3);
            Skew_Grstance_lh(1,0) =  Trsf_GenFoot_Lhand(2,3);
            Skew_Grstance_lh(1,2) = -Trsf_GenFoot_Lhand(0,3);
            Skew_Grstance_lh(2,0) = -Trsf_GenFoot_Lhand(1,3);
            Skew_Grstance_lh(2,1) =  Trsf_GenFoot_Lhand(0,3);

            Grstance_lhand_VeloTwistMx.block(0,0, 3,3) = Trsf_GenFoot_Lhand.block(0,0, 3,3);
            Grstance_lhand_VeloTwistMx.block(0,3, 3,3) = Skew_Grstance_lh * Trsf_GenFoot_Lhand.block(0,0, 3,3);
            Grstance_lhand_VeloTwistMx.block(3,3, 3,3) = Trsf_GenFoot_Lhand.block(0,0, 3,3);

            Lhand_GenStanceLegVeloTwistMx = Grstance_lhand_VeloTwistMx;

    }

    return Lhand_GenStanceLegVeloTwistMx;

}



// Velocity twist matrix of a General stance foot frame (taken as the wolrd frame) wrt. the right hand frame (h_W_w)
MatrixXd Grasping::getRhand_GenStanceLegVeloTwistMx( iCub::iKin::iKinChain *right_Arm_Chain, 
                                                     iCub::iKin::iKinChain *LeftLegChain,
                                                     iCub::iKin::iKinChain *RightLegChain,
                                                     bool StanceLeg)
{
    yarp::sig::Vector RightHandPoseInRoot;  // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    Eigen::MatrixXd   Trsf_RHandInRoot;     // Homogeneous transformation of the left hand frame wrt. the root frame
  
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    Eigen::MatrixXd   Trsf_LeftFootInRoot;  // Homogeneous transformation of the left leg frame wrt. the root frame
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame
    Eigen::MatrixXd   Trsf_RightFootInRoot; // Homogeneous transformation of the right leg frame wrt. the root frame

    

    RightHandPoseInRoot = right_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(RightHandPoseInRoot.subVector(3,6));
   
    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));



    Trsf_RHandInRoot.resize(4,4);       Trsf_RHandInRoot.setZero(4,4);       Trsf_RHandInRoot(3,3)     = 1.0;
    Trsf_RHandInRoot(0,3) = RightHandPoseInRoot[0];
    Trsf_RHandInRoot(1,3) = RightHandPoseInRoot[1];
    Trsf_RHandInRoot(2,3) = RightHandPoseInRoot[2];
    
    Trsf_LeftFootInRoot.resize(4,4);    Trsf_LeftFootInRoot.setZero(4,4);    Trsf_LeftFootInRoot(3,3)  = 1.0;
    Trsf_LeftFootInRoot(0,3) = LeftFootPoseInRoot[0];
    Trsf_LeftFootInRoot(1,3) = LeftFootPoseInRoot[1];
    Trsf_LeftFootInRoot(2,3) = LeftFootPoseInRoot[2];

    Trsf_RightFootInRoot.resize(4,4);   Trsf_RightFootInRoot.setZero(4,4);   Trsf_RightFootInRoot(3,3) = 1.0;
    Trsf_RightFootInRoot(0,3) = RightFootPoseInRoot[0];
    Trsf_RightFootInRoot(1,3) = RightFootPoseInRoot[1];
    Trsf_RightFootInRoot(2,3) = RightFootPoseInRoot[2];

    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
            Trsf_RHandInRoot(row,col)     = Rot_HandInRoot(row,col);
            Trsf_LeftFootInRoot(row,col)  = Rot_LeftFootInRoot(row,col);
            Trsf_RightFootInRoot(row,col) = Rot_RightFootInRoot(row,col);
        }
    }

    MatrixXd Trsf_GenFoot_icubFoot(4,4);
             Trsf_GenFoot_icubFoot.setZero(4,4);    Trsf_GenFoot_icubFoot(3,3) = 1.0;
             Trsf_GenFoot_icubFoot.block(0,0, 3,3) = Rot_icubFoot_GenFoot.transpose();


    MatrixXd Trsf_GenFoot_Rhand(4,4);

    if(StanceLeg)
    {
        // left hand with left leg as stance foot
        Trsf_GenFoot_Rhand = Trsf_RHandInRoot.inverse() * Trsf_LeftFootInRoot * Trsf_GenFoot_icubFoot;

        Matrix3d Skew_Glstance_rh;
                 Skew_Glstance_rh.setZero(3,3);

            Skew_Glstance_rh(0,1) = -Trsf_GenFoot_Rhand(2,3);
            Skew_Glstance_rh(0,2) =  Trsf_GenFoot_Rhand(1,3);
            Skew_Glstance_rh(1,0) =  Trsf_GenFoot_Rhand(2,3);
            Skew_Glstance_rh(1,2) = -Trsf_GenFoot_Rhand(0,3);
            Skew_Glstance_rh(2,0) = -Trsf_GenFoot_Rhand(1,3);
            Skew_Glstance_rh(2,1) =  Trsf_GenFoot_Rhand(0,3);

            Glstance_rhand_VeloTwistMx.block(0,0, 3,3) = Trsf_GenFoot_Rhand.block(0,0, 3,3);
            Glstance_rhand_VeloTwistMx.block(0,3, 3,3) = Skew_Glstance_rh * Trsf_GenFoot_Rhand.block(0,0, 3,3);
            Glstance_rhand_VeloTwistMx.block(3,3, 3,3) = Trsf_GenFoot_Rhand.block(0,0, 3,3);

            Rhand_GenStanceLegVeloTwistMx = Glstance_rhand_VeloTwistMx;


    }
    else
    {
        // left hand with right leg as stance foot   
        Trsf_GenFoot_Rhand = Trsf_RHandInRoot.inverse() * Trsf_RightFootInRoot * Trsf_GenFoot_icubFoot;

        Matrix3d Skew_Grstance_rh;
             Skew_Grstance_rh.setZero(3,3);

            Skew_Grstance_rh(0,1) = -Trsf_GenFoot_Rhand(2,3);
            Skew_Grstance_rh(0,2) =  Trsf_GenFoot_Rhand(1,3);
            Skew_Grstance_rh(1,0) =  Trsf_GenFoot_Rhand(2,3);
            Skew_Grstance_rh(1,2) = -Trsf_GenFoot_Rhand(0,3);
            Skew_Grstance_rh(2,0) = -Trsf_GenFoot_Rhand(1,3);
            Skew_Grstance_rh(2,1) =  Trsf_GenFoot_Rhand(0,3);

            Grstance_rhand_VeloTwistMx.block(0,0, 3,3) = Trsf_GenFoot_Rhand.block(0,0, 3,3);
            Grstance_rhand_VeloTwistMx.block(0,3, 3,3) = Skew_Grstance_rh * Trsf_GenFoot_Rhand.block(0,0, 3,3);
            Grstance_rhand_VeloTwistMx.block(3,3, 3,3) = Trsf_GenFoot_Rhand.block(0,0, 3,3);

            Rhand_GenStanceLegVeloTwistMx = Grstance_rhand_VeloTwistMx;

    }

    return Rhand_GenStanceLegVeloTwistMx;

}

// Task Jacobian of the left hand associated to the robot root link with left stance leg
MatrixXd Grasping::getLhand_virtBase_TaskJacobian( iCub::iKin::iKinChain *left_Arm_Chain, 
                                                   iCub::iKin::iKinChain *LeftLegChain,
                                                   iCub::iKin::iKinChain *RightLegChain,
                                                   bool StanceLeg)
{
    
    yarp::sig::Vector LeftHandPoseInRoot;   // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    yarp::sig::Matrix Jacobian_lh_rt_yarp;  // Jacobian of the left hand wrt. the root frame (as yarp matrix)
    // 
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame

    

    LeftHandPoseInRoot  = left_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(LeftHandPoseInRoot.subVector(3,6));
    Jacobian_lh_rt_yarp = left_Arm_Chain->GeoJacobian();

    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));

    // Eigen matrix of left hand Jacobian in root 
    Jacobian_Lhand_root.resize(Jacobian_lh_rt_yarp.rows(), Jacobian_lh_rt_yarp.cols());

    for (int row=0; row<Jacobian_lh_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_lh_rt_yarp.cols(); col++)
        {
            Jacobian_Lhand_root(row, col) = Jacobian_lh_rt_yarp(row, col);
        }
    }

   
    MatrixXd ll_R_rt,       // rotation Mx of the root frame wrt. the left foot frame
             Gll_R_rt;      // rotation Mx of the root frame wrt. the General left foot frame
    
             ll_R_rt.resize(3,3);
             ll_R_rt.setZero(3,3);

    MatrixXd rl_R_rt,       // rotation Mx of the root frame wrt. the  right foot frame
             Grl_R_rt;      // rotation Mx of the root frame wrt. the General right foot frame

             rl_R_rt.resize(3,3);
             rl_R_rt.setZero(3,3);

    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
             ll_R_rt(row,col) = Rot_LeftFootInRoot(col,row);
             rl_R_rt(row,col) = Rot_RightFootInRoot(col,row);
        }
    }

    // Rotation Mx of the root frame wrt to a general left foot frame 
    Gll_R_rt = Rot_icubFoot_GenFoot * ll_R_rt;
    
    Rot6x6_root_Glstance.block(0,0, 3,3) = Gll_R_rt;
    Rot6x6_root_Glstance.block(3,3, 3,3) = Gll_R_rt;

     // Rotation Mx of the root frame wrt to a general right foot frame 
    Grl_R_rt = Rot_icubFoot_GenFoot * rl_R_rt;

    Rot6x6_root_Grstance.block(0,0, 3,3) = Grl_R_rt;
    Rot6x6_root_Grstance.block(3,3, 3,3) = Grl_R_rt;

        

    // Mapping between the left hand Veclocity and the left hand joints velocoties and the 
    // the velocity of the virtual base (assumed attached to the root link)
        Lhand_virtBase_TaskJacobian.resize(6, Jacobian_Lhand_root.cols() + 6);

    if (StanceLeg) // left stance leg
    {
        
        // translation left
        Vector3d rt_Posi_lh, Glstance_Posi_lh;
             
        rt_Posi_lh(0) = LeftHandPoseInRoot[0];
        rt_Posi_lh(1) = LeftHandPoseInRoot[1];
        rt_Posi_lh(2) = LeftHandPoseInRoot[2];

        Glstance_Posi_lh   = Gll_R_rt * rt_Posi_lh;

        Matrix3d Skew_lh_Glstance;
                 Skew_lh_Glstance.setZero(3,3);

        Skew_lh_Glstance(0,1) = -Glstance_Posi_lh(2);
        Skew_lh_Glstance(0,2) =  Glstance_Posi_lh(1);
        Skew_lh_Glstance(1,0) =  Glstance_Posi_lh(2);
        Skew_lh_Glstance(1,2) = -Glstance_Posi_lh(0);
        Skew_lh_Glstance(2,0) = -Glstance_Posi_lh(1);
        Skew_lh_Glstance(2,1) =  Glstance_Posi_lh(0);

        // Mapping between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        RootVeloTwistMx_lhand_lleg.block(0,0, 3,3) =  MatrixXd::Identity(3,3);
        RootVeloTwistMx_lhand_lleg.block(0,3, 3,3) = -Skew_lh_Glstance;
        RootVeloTwistMx_lhand_lleg.block(3,3, 3,3) =  MatrixXd::Identity(3,3);


        Lhand_virtBase_TaskJacobian.block(0,0, 6, Jacobian_Lhand_root.cols()) =  Rot6x6_root_Glstance * Jacobian_Lhand_root;
        Lhand_virtBase_TaskJacobian.block(0, Jacobian_Lhand_root.cols(), 6,6) =  RootVeloTwistMx_lhand_lleg;
    }
    else
    {
        // translation right
        Vector3d rt_Posi_lh, Grstance_Posi_lh;
             
        Grstance_Posi_lh   = Grl_R_rt * rt_Posi_lh;

        Matrix3d Skew_lh_Grstance;
                 Skew_lh_Grstance.setZero(3,3);

        Skew_lh_Grstance(0,1) = -Grstance_Posi_lh(2);
        Skew_lh_Grstance(0,2) =  Grstance_Posi_lh(1);
        Skew_lh_Grstance(1,0) =  Grstance_Posi_lh(2);
        Skew_lh_Grstance(1,2) = -Grstance_Posi_lh(0);
        Skew_lh_Grstance(2,0) = -Grstance_Posi_lh(1);
        Skew_lh_Grstance(2,1) =  Grstance_Posi_lh(0);

        // Mapping between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices

        RootVeloTwistMx_lhand_rleg.block(0,0, 3,3) =  MatrixXd::Identity(3,3);
        RootVeloTwistMx_lhand_rleg.block(0,3, 3,3) = -Skew_lh_Grstance;
        RootVeloTwistMx_lhand_rleg.block(3,3, 3,3) =  MatrixXd::Identity(3,3);


        Lhand_virtBase_TaskJacobian.block(0,0, 6, Jacobian_Lhand_root.cols()) =  Rot6x6_root_Grstance * Jacobian_Lhand_root;
        Lhand_virtBase_TaskJacobian.block(0, Jacobian_Lhand_root.cols(), 6,6) =  RootVeloTwistMx_lhand_rleg;
    }
      

    return Lhand_virtBase_TaskJacobian;

}


// Task Jacobian of the right hand associated to the robot root link with left stance leg
MatrixXd Grasping::getRhand_virtBase_TaskJacobian( iCub::iKin::iKinChain *right_Arm_Chain, 
                                                   iCub::iKin::iKinChain *LeftLegChain,
                                                   iCub::iKin::iKinChain *RightLegChain,
                                                   bool StanceLeg)
{
    
    // right hand
    yarp::sig::Vector RightHandPoseInRoot;  // Pose of right hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the right hand frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rh_rt_yarp;  // Jacobian of the right hand wrt. the root frame (as yarp matrix)

    // left leg
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    // right leg
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame

    

    RightHandPoseInRoot = right_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(RightHandPoseInRoot.subVector(3,6));
    Jacobian_rh_rt_yarp = right_Arm_Chain->GeoJacobian();

    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));

    // Eigen matrix of left hand Jacobian in root 
    Jacobian_Rhand_root.resize(Jacobian_rh_rt_yarp.rows(), Jacobian_rh_rt_yarp.cols());

    for (int row=0; row<Jacobian_rh_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_rh_rt_yarp.cols(); col++)
        {
            Jacobian_Rhand_root(row, col) = Jacobian_rh_rt_yarp(row, col);
        }
    }

   
    MatrixXd ll_R_rt,       // rotation Mx of the root frame wrt. the left foot frame
             Gll_R_rt;      // rotation Mx of the root frame wrt. the General left foot frame
    
             ll_R_rt.resize(3,3);
             ll_R_rt.setZero(3,3);

    MatrixXd rl_R_rt,       // rotation Mx of the root frame wrt. the  right foot frame
             Grl_R_rt;      // rotation Mx of the root frame wrt. the General right foot frame

             rl_R_rt.resize(3,3);
             rl_R_rt.setZero(3,3);

    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
             ll_R_rt(row,col) = Rot_LeftFootInRoot(col,row);
             rl_R_rt(row,col) = Rot_RightFootInRoot(col,row);
        }
    }

    // Rotation Mx of the root frame wrt to a general left foot frame 
    Gll_R_rt = Rot_icubFoot_GenFoot * ll_R_rt;
    
    Rot6x6_root_Glstance.block(0,0, 3,3) = Gll_R_rt;
    Rot6x6_root_Glstance.block(3,3, 3,3) = Gll_R_rt;

     // Rotation Mx of the root frame wrt to a general right foot frame 
    Grl_R_rt = Rot_icubFoot_GenFoot * rl_R_rt;

    Rot6x6_root_Grstance.block(0,0, 3,3) = Grl_R_rt;
    Rot6x6_root_Grstance.block(3,3, 3,3) = Grl_R_rt;

        

    // Mapping between the left hand Veclocity and the left hand joints velocoties and the 
    // the velocity of the virtual base (assumed attached to the root link)
        Rhand_virtBase_TaskJacobian.resize(6, Jacobian_Rhand_root.cols() + 6);

    if (StanceLeg) // left stance leg
    {
        
        // translation left
        Vector3d rt_Posi_rh, Glstance_Posi_rh;
             
        rt_Posi_rh(0) = RightHandPoseInRoot[0];
        rt_Posi_rh(1) = RightHandPoseInRoot[1];
        rt_Posi_rh(2) = RightHandPoseInRoot[2];

        Glstance_Posi_rh   = Gll_R_rt * rt_Posi_rh;

        Matrix3d Skew_rh_Glstance;
                 Skew_rh_Glstance.setZero(3,3);

        Skew_rh_Glstance(0,1) = -Glstance_Posi_rh(2);
        Skew_rh_Glstance(0,2) =  Glstance_Posi_rh(1);
        Skew_rh_Glstance(1,0) =  Glstance_Posi_rh(2);
        Skew_rh_Glstance(1,2) = -Glstance_Posi_rh(0);
        Skew_rh_Glstance(2,0) = -Glstance_Posi_rh(1);
        Skew_rh_Glstance(2,1) =  Glstance_Posi_rh(0);

        // Mapping between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        RootVeloTwistMx_rhand_lleg.block(0,0, 3,3) =  MatrixXd::Identity(3,3);
        RootVeloTwistMx_rhand_lleg.block(0,3, 3,3) = -Skew_rh_Glstance;
        RootVeloTwistMx_rhand_lleg.block(3,3, 3,3) =  MatrixXd::Identity(3,3);


        Rhand_virtBase_TaskJacobian.block(0,0, 6, Jacobian_Rhand_root.cols()) =  Rot6x6_root_Glstance * Jacobian_Rhand_root;
        Rhand_virtBase_TaskJacobian.block(0, Jacobian_Rhand_root.cols(), 6,6) =  RootVeloTwistMx_rhand_lleg;
    }
    else
    {
        // translation right
        Vector3d rt_Posi_rh, Grstance_Posi_rh;
             
        Grstance_Posi_rh   = Grl_R_rt * rt_Posi_rh;

        Matrix3d Skew_rh_Grstance;
                 Skew_rh_Grstance.setZero(3,3);

        Skew_rh_Grstance(0,1) = -Grstance_Posi_rh(2);
        Skew_rh_Grstance(0,2) =  Grstance_Posi_rh(1);
        Skew_rh_Grstance(1,0) =  Grstance_Posi_rh(2);
        Skew_rh_Grstance(1,2) = -Grstance_Posi_rh(0);
        Skew_rh_Grstance(2,0) = -Grstance_Posi_rh(1);
        Skew_rh_Grstance(2,1) =  Grstance_Posi_rh(0);

        // Mapping between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices

        RootVeloTwistMx_rhand_rleg.block(0,0, 3,3) =  MatrixXd::Identity(3,3);
        RootVeloTwistMx_rhand_rleg.block(0,3, 3,3) = -Skew_rh_Grstance;
        RootVeloTwistMx_rhand_rleg.block(3,3, 3,3) =  MatrixXd::Identity(3,3);


        Rhand_virtBase_TaskJacobian.block(0,0, 6, Jacobian_Rhand_root.cols()) =  Rot6x6_root_Grstance * Jacobian_Rhand_root;
        Rhand_virtBase_TaskJacobian.block(0, Jacobian_Rhand_root.cols(), 6,6) =  RootVeloTwistMx_rhand_rleg;
    }
      

    return Rhand_virtBase_TaskJacobian;

}


// Task jacobian of left hand wrt. the stance legs
MatrixXd Grasping::getLhand_StanceLeg_TaskJacobian( iCub::iKin::iKinChain *left_Arm_Chain, 
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    bool StanceLeg)
{
    
    int nlh_jts = left_Arm_Chain->getDOF();

    MatrixXd lh_VBase_TaskJac;

    lh_VBase_TaskJac = Grasping::getLhand_virtBase_TaskJacobian( left_Arm_Chain, 
				                                                 LeftLegChain,
				                                                 RightLegChain,
				                                                 StanceLeg);


    // 
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_ll_rt_yarp;  // Jacobian of the left leg wrt. the root frame (as yarp matrix)
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rl_rt_yarp;  // Jacobian of the right leg wrt. the root frame (as yarp matrix)

    

    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));
    Jacobian_ll_rt_yarp = LeftLegChain->GeoJacobian();

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));
    Jacobian_rl_rt_yarp = RightLegChain->GeoJacobian();

            
        // 2) the velocity of the stance leg
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Eigen matrix of left leg Jacobian in root 
	    Jacobian_Lleg_root.resize(Jacobian_ll_rt_yarp.rows(), Jacobian_ll_rt_yarp.cols());

	    for (int row=0; row<Jacobian_ll_rt_yarp.rows(); row++)
	    {
	        for(int col=0; col<Jacobian_ll_rt_yarp.cols(); col++)
	        {
	            Jacobian_Lleg_root(row, col) = Jacobian_ll_rt_yarp(row, col);
	        }
	    }

	    // Eigen matrix of right leg Jacobian in root 
	    Jacobian_Rleg_root.resize(Jacobian_rl_rt_yarp.rows(), Jacobian_rl_rt_yarp.cols());

	    for (int row=0; row<Jacobian_rl_rt_yarp.rows(); row++)
	    {
	        for(int col=0; col<Jacobian_rl_rt_yarp.cols(); col++)
	        {
	            Jacobian_Rleg_root(row, col) = Jacobian_rl_rt_yarp(row, col);
	        }
	    }

	    // Mapping between the left hand Veclocity and the left hand joints velocoties and 
        // 2) the velocity of the stance leg
        Lhand_StanceLeg_TaskJacobian.resize(6, nlh_jts + Jacobian_Lleg_root.cols());


    MatrixXd ll_R_rt,       // rotation Mx of the root frame wrt. the left foot frame
             Gll_R_rt;      // rotation Mx of the root frame wrt. the General left foot frame
    
             ll_R_rt.resize(3,3);
             ll_R_rt.setZero(3,3);

    MatrixXd rl_R_rt,       // rotation Mx of the root frame wrt. the  right foot frame
             Grl_R_rt;      // rotation Mx of the root frame wrt. the General right foot frame

             rl_R_rt.resize(3,3);
             rl_R_rt.setZero(3,3);

    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
             ll_R_rt(row,col) = Rot_LeftFootInRoot(col,row);
             rl_R_rt(row,col) = Rot_RightFootInRoot(col,row);
        }
    }

    // Rotation Mx of the root frame wrt to a general left foot frame 
    Gll_R_rt = Rot_icubFoot_GenFoot * ll_R_rt;
    
     // Rotation Mx of the root frame wrt to a general right foot frame 
    Grl_R_rt = Rot_icubFoot_GenFoot * rl_R_rt;


    if (StanceLeg) // left stance leg
    {
        
        // translation left leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_ll, Glstance_Posi_ll;
         
        rt_Posi_ll(0) = LeftFootPoseInRoot[0];
        rt_Posi_ll(1) = LeftFootPoseInRoot[1];
        rt_Posi_ll(2) = LeftFootPoseInRoot[2];

        Glstance_Posi_ll  = Gll_R_rt * rt_Posi_ll;

        Matrix3d Skew_ll_Glstance;
                 Skew_ll_Glstance.setZero(3,3);

        Skew_ll_Glstance(0,1) = -Glstance_Posi_ll(2);
        Skew_ll_Glstance(0,2) =  Glstance_Posi_ll(1);
        Skew_ll_Glstance(1,0) =  Glstance_Posi_ll(2);
        Skew_ll_Glstance(1,2) = -Glstance_Posi_ll(0);
        Skew_ll_Glstance(2,0) = -Glstance_Posi_ll(1);
        Skew_ll_Glstance(2,1) =  Glstance_Posi_ll(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_LeftStanceLeg_Jacobian.block(0,0, 3,3) = Gll_R_rt;
        Root_LeftStanceLeg_Jacobian.block(0,3, 3,3) = Skew_ll_Glstance * Gll_R_rt;
        Root_LeftStanceLeg_Jacobian.block(3,3, 3,3) = Gll_R_rt;

        //
        //
        Lhand_StanceLeg_TaskJacobian.block(0,0, 6, nlh_jts) =  lh_VBase_TaskJac.block(0,0, 6, nlh_jts);
        Lhand_StanceLeg_TaskJacobian.block(0, nlh_jts, 6, Jacobian_Lleg_root.cols()) = -RootVeloTwistMx_lhand_lleg * Root_LeftStanceLeg_Jacobian * Jacobian_Lleg_root;      


    }
    else
    {

        // translation right leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_rl, Grstance_Posi_rl;
         
        rt_Posi_rl(0) = RightFootPoseInRoot[0];
        rt_Posi_rl(1) = RightFootPoseInRoot[1];
        rt_Posi_rl(2) = RightFootPoseInRoot[2];

        Grstance_Posi_rl   = Grl_R_rt * rt_Posi_rl;

        Matrix3d Skew_rl_Grstance;
             Skew_rl_Grstance.setZero(3,3);

        Skew_rl_Grstance(0,1) = -Grstance_Posi_rl(2);
        Skew_rl_Grstance(0,2) =  Grstance_Posi_rl(1);
        Skew_rl_Grstance(1,0) =  Grstance_Posi_rl(2);
        Skew_rl_Grstance(1,2) = -Grstance_Posi_rl(0);
        Skew_rl_Grstance(2,0) = -Grstance_Posi_rl(1);
        Skew_rl_Grstance(2,1) =  Grstance_Posi_rl(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_RightStanceLeg_Jacobian.block(0,0, 3,3) = Grl_R_rt;
        Root_RightStanceLeg_Jacobian.block(0,3, 3,3) = Skew_rl_Grstance * Grl_R_rt;
        Root_RightStanceLeg_Jacobian.block(3,3, 3,3) = Grl_R_rt;

        //
        //
        Lhand_StanceLeg_TaskJacobian.block(0,0, 6, nlh_jts) =  lh_VBase_TaskJac.block(0,0, 6, nlh_jts);
        Lhand_StanceLeg_TaskJacobian.block(0, nlh_jts, 6, Jacobian_Rleg_root.cols()) = -RootVeloTwistMx_lhand_rleg * Root_RightStanceLeg_Jacobian * Jacobian_Rleg_root;


    }
      

    return Lhand_StanceLeg_TaskJacobian;
}


// Task jacobian of right hand wrt. the stance legs (with General stance foot taken as world frame)
MatrixXd Grasping::getRhand_StanceLeg_TaskJacobian( iCub::iKin::iKinChain *right_Arm_Chain, 
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    bool StanceLeg)
{
    
	int nrh_jts = right_Arm_Chain->getDOF();

    MatrixXd rh_VBase_TaskJac;

    rh_VBase_TaskJac = Grasping::getRhand_virtBase_TaskJacobian( right_Arm_Chain, 
                                                   				 LeftLegChain,
                                                   				 RightLegChain,
                                                   				 StanceLeg);

    // 
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_ll_rt_yarp;  // Jacobian of the left leg wrt. the root frame (as yarp matrix)
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rl_rt_yarp;  // Jacobian of the right leg wrt. the root frame (as yarp matrix)

    

    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));
    Jacobian_ll_rt_yarp = LeftLegChain->GeoJacobian();

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));
    Jacobian_rl_rt_yarp = RightLegChain->GeoJacobian();

    
    // Eigen matrix of left leg Jacobian in root 
    Jacobian_Lleg_root.resize(Jacobian_ll_rt_yarp.rows(), Jacobian_ll_rt_yarp.cols());

    for (int row=0; row<Jacobian_ll_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_ll_rt_yarp.cols(); col++)
        {
            Jacobian_Lleg_root(row, col) = Jacobian_ll_rt_yarp(row, col);
        }
    }

    // Eigen matrix of right leg Jacobian in root 
    Jacobian_Rleg_root.resize(Jacobian_rl_rt_yarp.rows(), Jacobian_rl_rt_yarp.cols());

    for (int row=0; row<Jacobian_rl_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_rl_rt_yarp.cols(); col++)
        {
            Jacobian_Rleg_root(row, col) = Jacobian_rl_rt_yarp(row, col);
        }
    }

    // 
    MatrixXd ll_R_rt,       // rotation Mx of the root frame wrt. the left foot frame
             Gll_R_rt;      // rotation Mx of the root frame wrt. the General left foot frame
    
             ll_R_rt.resize(3,3);
             ll_R_rt.setZero(3,3);

    MatrixXd rl_R_rt,       // rotation Mx of the root frame wrt. the  right foot frame
             Grl_R_rt;      // rotation Mx of the root frame wrt. the General right foot frame

             rl_R_rt.resize(3,3);
             rl_R_rt.setZero(3,3);

    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
             ll_R_rt(row,col) = Rot_LeftFootInRoot(col,row);
             rl_R_rt(row,col) = Rot_RightFootInRoot(col,row);
        }
    }

    // Rotation Mx of the root frame wrt to a general left foot frame 
    Gll_R_rt = Rot_icubFoot_GenFoot * ll_R_rt;
    
     // Rotation Mx of the root frame wrt to a general right foot frame 
    Grl_R_rt = Rot_icubFoot_GenFoot * rl_R_rt;
      

    // Mapping between the left hand Veclocity and the right hand joints velocoties and 
        // 2) the velocity of the stance leg
        Rhand_StanceLeg_TaskJacobian.resize(6, Jacobian_Rhand_root.cols() + Jacobian_Lleg_root.cols());

    if (StanceLeg) // left stance leg
    {
        
        // translation left leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_ll, Glstance_Posi_ll;
         
        rt_Posi_ll(0) = LeftFootPoseInRoot[0];
        rt_Posi_ll(1) = LeftFootPoseInRoot[1];
        rt_Posi_ll(2) = LeftFootPoseInRoot[2];

        Glstance_Posi_ll  = Gll_R_rt * rt_Posi_ll;

        Matrix3d Skew_ll_Glstance;
                 Skew_ll_Glstance.setZero(3,3);

        Skew_ll_Glstance(0,1) = -Glstance_Posi_ll(2);
        Skew_ll_Glstance(0,2) =  Glstance_Posi_ll(1);
        Skew_ll_Glstance(1,0) =  Glstance_Posi_ll(2);
        Skew_ll_Glstance(1,2) = -Glstance_Posi_ll(0);
        Skew_ll_Glstance(2,0) = -Glstance_Posi_ll(1);
        Skew_ll_Glstance(2,1) =  Glstance_Posi_ll(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_LeftStanceLeg_Jacobian.block(0,0, 3,3) = Gll_R_rt;
        Root_LeftStanceLeg_Jacobian.block(0,3, 3,3) = Skew_ll_Glstance * Gll_R_rt;
        Root_LeftStanceLeg_Jacobian.block(3,3, 3,3) = Gll_R_rt;

        //
        //
        Rhand_StanceLeg_TaskJacobian.block(0,0, 6, nrh_jts) = rh_VBase_TaskJac.block(0,0, 6, nrh_jts);
        Rhand_StanceLeg_TaskJacobian.block(0, nrh_jts, 6, Jacobian_Lleg_root.cols()) = -RootVeloTwistMx_rhand_lleg * Root_LeftStanceLeg_Jacobian * Jacobian_Lleg_root;    


    }
    else
    {
        // translation right leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_rl, Grstance_Posi_rl;
         
        rt_Posi_rl(0) = RightFootPoseInRoot[0];
        rt_Posi_rl(1) = RightFootPoseInRoot[1];
        rt_Posi_rl(2) = RightFootPoseInRoot[2];

        Grstance_Posi_rl   = Grl_R_rt * rt_Posi_rl;

        Matrix3d Skew_rl_Grstance;
             Skew_rl_Grstance.setZero(3,3);

        Skew_rl_Grstance(0,1) = -Grstance_Posi_rl(2);
        Skew_rl_Grstance(0,2) =  Grstance_Posi_rl(1);
        Skew_rl_Grstance(1,0) =  Grstance_Posi_rl(2);
        Skew_rl_Grstance(1,2) = -Grstance_Posi_rl(0);
        Skew_rl_Grstance(2,0) = -Grstance_Posi_rl(1);
        Skew_rl_Grstance(2,1) =  Grstance_Posi_rl(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_RightStanceLeg_Jacobian.block(0,0, 3,3) = Grl_R_rt;
        Root_RightStanceLeg_Jacobian.block(0,3, 3,3) = Skew_rl_Grstance * Grl_R_rt;
        Root_RightStanceLeg_Jacobian.block(3,3, 3,3) = Grl_R_rt;

        //
        //
        Rhand_StanceLeg_TaskJacobian.block(0,0, 6, nrh_jts) = rh_VBase_TaskJac.block(0,0, 6, nrh_jts);
        Rhand_StanceLeg_TaskJacobian.block(0, nrh_jts, 6, Jacobian_Rleg_root.cols()) = -RootVeloTwistMx_rhand_rleg * Root_RightStanceLeg_Jacobian * Jacobian_Rleg_root;


    }
      

    return Rhand_StanceLeg_TaskJacobian;
}

//
// Grasping Jacobian of the left hand wrt. the General stance leg
MatrixXd Grasping::getGraspJacobianLhand_GenStanceLeg(  MatrixXd d_H_c,
														iCub::iKin::iKinChain *left_Arm_Chain, 
                                                        iCub::iKin::iKinChain *LeftLegChain,
                                                        iCub::iKin::iKinChain *RightLegChain,
                                                        bool StanceLeg)
{
    
    MatrixXd L_eta_lhand;
    MatrixXd W_GStanceLeg_lhand;
    MatrixXd J_task_lhand_GenStanceLeg;

    L_eta_lhand = Grasping::getInteractionMxForAxisAngle(d_H_c);

    W_GStanceLeg_lhand = Grasping::getLhand_GenStanceLegVeloTwistMx(left_Arm_Chain, 
                                                                 LeftLegChain,
                                                                 RightLegChain,
                                                                 StanceLeg);

    J_task_lhand_GenStanceLeg = Grasping::getLhand_StanceLeg_TaskJacobian( left_Arm_Chain, 
                                                    					    LeftLegChain,
                                                    						RightLegChain,
                                                    						StanceLeg);

    GraspJacobianLhand_GenStanceLeg = L_eta_lhand * W_GStanceLeg_lhand * J_task_lhand_GenStanceLeg;

    return GraspJacobianLhand_GenStanceLeg; 

}

// Grasping Jacobian of the right hand wrt. the General stance leg
MatrixXd Grasping::getGraspJacobianRhand_GenStanceLeg(  MatrixXd d_H_c,
														iCub::iKin::iKinChain *right_Arm_Chain, 
                                                        iCub::iKin::iKinChain *LeftLegChain,
                                                        iCub::iKin::iKinChain *RightLegChain,
                                                        bool StanceLeg)
{

    MatrixXd L_eta_rhand;
    MatrixXd W_GStanceLeg_rhand;
    MatrixXd J_task_rhand_GenStanceLeg;

    L_eta_rhand = Grasping::getInteractionMxForAxisAngle(d_H_c);

    W_GStanceLeg_rhand = Grasping::getRhand_GenStanceLegVeloTwistMx( right_Arm_Chain, 
                                                                      LeftLegChain,
                                                                      RightLegChain,
                                                                      StanceLeg);

    J_task_rhand_GenStanceLeg = Grasping::getRhand_StanceLeg_TaskJacobian(  right_Arm_Chain, 
                                                                            LeftLegChain,
                                                                            RightLegChain,
                                                                            StanceLeg);

    GraspJacobianRhand_GenStanceLeg = L_eta_rhand * W_GStanceLeg_rhand * J_task_rhand_GenStanceLeg;

    return GraspJacobianRhand_GenStanceLeg; 
}
//
//
// left hand joint velocities for the grasping task  (with Gen stance leg as world frame)
VectorXd Grasping::get_lh_GraspJtsVelocity_GStance( MatrixXd dl_H_cl,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    VectorXd e_grasp;
    MatrixXd J_grap_Lhand_Gleg;
    MatrixXd J_grasp_Lhand;
    MatrixXd J_grasp_stanceleg;
    VectorXd q_dot_Lhand;

    e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);

    
    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        
        J_grap_Lhand_Gleg = Grasping::getGraspJacobianLhand_GenStanceLeg( dl_H_cl,
																		  left_Arm_Chain, 
                                                        				  LeftLegChain,
                                                        				  RightLegChain,
                                                        				  StanceLeg);

        J_grasp_Lhand     = J_grap_Lhand_Gleg.block(0, 0, 6, n_lh);
        J_grasp_stanceleg = J_grap_Lhand_Gleg.block(0, n_lh, 6, n_ll);

        q_dot_Lhand = MxPsdInv.pseudoInverse(J_grasp_Lhand) * (-1.0 * graspGain_lh.asDiagonal() * e_grasp - J_grasp_stanceleg * q_dot_lleg);

    }
    else
    {
        J_grap_Lhand_Gleg = Grasping::getGraspJacobianLhand_GenStanceLeg( dl_H_cl,
																		  left_Arm_Chain, 
                                                        				  LeftLegChain,
                                                        				  RightLegChain,
                                                        				  StanceLeg);
        
        J_grasp_Lhand     = J_grap_Lhand_Gleg.block(0, 0, 6, n_lh);
        J_grasp_stanceleg = J_grap_Lhand_Gleg.block(0, n_lh, 6, n_ll);

        q_dot_Lhand = MxPsdInv.pseudoInverse(J_grasp_Lhand) * (-1.0 *graspGain_lh.asDiagonal() * e_grasp - J_grasp_stanceleg * q_dot_rleg);

    }

    return q_dot_Lhand;
}


// right hand joint velocities for the grasping task  (with Gen stance leg as world frame) 
VectorXd Grasping::get_rh_GraspJtsVelocity_GStance( MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{

    VectorXd e_grasp;
    MatrixXd J_grap_Rhand_Gleg;
    MatrixXd J_grasp_Rhand;
    MatrixXd J_grasp_stanceleg;
    VectorXd q_dot_Rhand;

    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();

    e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);

    
    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        J_grap_Rhand_Gleg = Grasping::getGraspJacobianRhand_GenStanceLeg( dr_H_cr,
																		  right_Arm_Chain, 
                                                        				  LeftLegChain,
                                                        				  RightLegChain,
                                                        				  StanceLeg);

        // cout << " J_grap_Rhand_Gleg \n" << J_grap_Rhand_Gleg << endl;

        J_grasp_Rhand     = J_grap_Rhand_Gleg.block(0, 0, 6, n_rh);
        J_grasp_stanceleg = J_grap_Rhand_Gleg.block(0, n_rh, 6, n_ll);

        q_dot_Rhand = MxPsdInv.pseudoInverse(J_grasp_Rhand) * (-1.0 *graspGain_rh.asDiagonal() * e_grasp - J_grasp_stanceleg * q_dot_lleg);

    }
    else
    {
        J_grap_Rhand_Gleg = Grasping::getGraspJacobianRhand_GenStanceLeg( dr_H_cr,
																		  right_Arm_Chain, 
                                                        				  LeftLegChain,
                                                        				  RightLegChain,
                                                        				  StanceLeg);

        J_grasp_Rhand     = J_grap_Rhand_Gleg.block(0, 0, 6, n_rh);
        J_grasp_stanceleg = J_grap_Rhand_Gleg.block(0, n_rh, 6, n_ll);

        q_dot_Rhand = MxPsdInv.pseudoInverse(J_grasp_Rhand) * (-1.0 *graspGain_rh.asDiagonal() * e_grasp - J_grasp_stanceleg * q_dot_rleg);

    }

    return q_dot_Rhand;

}


//
// With respect to the world frame
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Velocity twist matrix of the wolrd frame wrt. the left hand frame (h_W_w)
MatrixXd Grasping::getLhand_World_VeloTwistMx(  MatrixXd wld_Trsf_root,
												iCub::iKin::iKinChain *left_Arm_Chain)
{
	yarp::sig::Vector LeftHandPoseInRoot;   // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    Eigen::MatrixXd Trsf_LHandInRoot;     // Homogeneous transformation of the left hand frame wrt. the root frame

    LeftHandPoseInRoot  = left_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(LeftHandPoseInRoot.subVector(3,6));
   
    
    Trsf_LHandInRoot.resize(4,4);    Trsf_LHandInRoot.setZero(4,4);   Trsf_LHandInRoot(3,3)     = 1.0;  

    Trsf_LHandInRoot(0,3) = LeftHandPoseInRoot[0];
    Trsf_LHandInRoot(1,3) = LeftHandPoseInRoot[1];
    Trsf_LHandInRoot(2,3) = LeftHandPoseInRoot[2];

   
    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
            Trsf_LHandInRoot(row,col)     = Rot_HandInRoot(row,col);
        }
    }

    cout << " Trsf_LHandInRoot is ; \n " << Trsf_LHandInRoot << endl;

    MatrixXd Trsf_World_Lhand(4,4), Trsf_Lhand_World(4,4);

    Trsf_Lhand_World = wld_Trsf_root * Trsf_LHandInRoot;
    // transformation from world frame to the left hand
    Trsf_World_Lhand = Trsf_Lhand_World.inverse();

    Matrix3d Skew_World_lh;
             Skew_World_lh.setZero(3,3);

    Skew_World_lh(0,1) = -Trsf_World_Lhand(2,3);
    Skew_World_lh(0,2) =  Trsf_World_Lhand(1,3);
    Skew_World_lh(1,0) =  Trsf_World_Lhand(2,3);
    Skew_World_lh(1,2) = -Trsf_World_Lhand(0,3);
    Skew_World_lh(2,0) = -Trsf_World_Lhand(1,3);
    Skew_World_lh(2,1) =  Trsf_World_Lhand(0,3);

    World_lhand_VeloTwistMx.block(0,0, 3,3) = Trsf_World_Lhand.block(0,0, 3,3);
    World_lhand_VeloTwistMx.block(0,3, 3,3) = 0. * Skew_World_lh * Trsf_World_Lhand.block(0,0, 3,3);
    World_lhand_VeloTwistMx.block(3,3, 3,3) = Trsf_World_Lhand.block(0,0, 3,3);

    Lhand_World_VeloTwistMx = World_lhand_VeloTwistMx;

    return Lhand_World_VeloTwistMx;

}

// Velocity twist matrix of a General stance foot frame (taken as the wolrd frame) wrt. the right hand frame (h_W_w)
MatrixXd Grasping::getRhand_World_VeloTwistMx( 	MatrixXd wld_Trsf_root,
												iCub::iKin::iKinChain *right_Arm_Chain)
{
	yarp::sig::Vector RightHandPoseInRoot;  // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    Eigen::MatrixXd Trsf_RHandInRoot;       // Homogeneous transformation of the left hand frame wrt. the root frame
      

    RightHandPoseInRoot = right_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(RightHandPoseInRoot.subVector(3,6));
   
    
    Trsf_RHandInRoot.resize(4,4);       Trsf_RHandInRoot.setZero(4,4);  Trsf_RHandInRoot(3,3)     = 1.0;
    Trsf_RHandInRoot(0,3) = RightHandPoseInRoot[0];
    Trsf_RHandInRoot(1,3) = RightHandPoseInRoot[1];
    Trsf_RHandInRoot(2,3) = RightHandPoseInRoot[2];    
    
    // extraction and transposing of the matrix data from yarp matrix to Eigien matrix
    for (int row=0; row<3; row++)
    {
        for (int col=0; col<3; col++)
        {
            Trsf_RHandInRoot(row,col)     = Rot_HandInRoot(row,col);
        }
    }

    cout << " Trsf_RHandInRoot is ; \n " << Trsf_RHandInRoot << endl;

    MatrixXd Trsf_World_Rhand(4,4), Trsf_Rhand_World(4,4);

    Trsf_Rhand_World = wld_Trsf_root * Trsf_RHandInRoot;
    // transformation from world frame to the right hand
    Trsf_World_Rhand = Trsf_Rhand_World.inverse();

    Matrix3d Skew_World_rh;
             Skew_World_rh.setZero(3,3);

    Skew_World_rh(0,1) = -Trsf_World_Rhand(2,3);
    Skew_World_rh(0,2) =  Trsf_World_Rhand(1,3);
    Skew_World_rh(1,0) =  Trsf_World_Rhand(2,3);
    Skew_World_rh(1,2) = -Trsf_World_Rhand(0,3);
    Skew_World_rh(2,0) = -Trsf_World_Rhand(1,3);
    Skew_World_rh(2,1) =  Trsf_World_Rhand(0,3);

    World_rhand_VeloTwistMx.block(0,0, 3,3) = Trsf_World_Rhand.block(0,0, 3,3);
    World_rhand_VeloTwistMx.block(0,3, 3,3) = 0.*Skew_World_rh * Trsf_World_Rhand.block(0,0, 3,3);
    World_rhand_VeloTwistMx.block(3,3, 3,3) = Trsf_World_Rhand.block(0,0, 3,3);

    Rhand_World_VeloTwistMx = World_rhand_VeloTwistMx;

    return Rhand_World_VeloTwistMx;

}

// Task Jacobian of the left hand associated to the robot root link with left stance leg
MatrixXd Grasping::getLhand_VBase_TskJac_World( MatrixXd wld_Trsf_root,
												iCub::iKin::iKinChain *left_Arm_Chain)
{
	//
	yarp::sig::Vector LeftHandPoseInRoot;   // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    yarp::sig::Matrix Jacobian_lh_rt_yarp;  // Jacobian of the left hand wrt. the root frame (as yarp matrix)
    //
    //   
    LeftHandPoseInRoot  = left_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(LeftHandPoseInRoot.subVector(3,6));
    Jacobian_lh_rt_yarp = left_Arm_Chain->GeoJacobian();

    // Eigen matrix of left hand Jacobian in root 
    Jacobian_Lhand_root.resize(Jacobian_lh_rt_yarp.rows(), Jacobian_lh_rt_yarp.cols());

    for (int row=0; row<Jacobian_lh_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_lh_rt_yarp.cols(); col++)
        {
            Jacobian_Lhand_root(row, col) = Jacobian_lh_rt_yarp(row, col);
        }
    } 
    
    // Rotation Mx of the root frame wrt to a general left foot frame 
    Rot6x6_root_World.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3); 
    Rot6x6_root_World.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);    

    // Mapping between the left hand Veclocity and the left hand joints velocoties and the 
    // the velocity of the virtual base (assumed attached to the root link)
    Lhand_VBase_TskJac_World.resize(6, Jacobian_Lhand_root.cols() + 6);

    //
    // translation left
    Vector3d rt_Posi_lh, World_Posi_lh;
             
    rt_Posi_lh(0) = LeftHandPoseInRoot[0];
    rt_Posi_lh(1) = LeftHandPoseInRoot[1];
    rt_Posi_lh(2) = LeftHandPoseInRoot[2];

    World_Posi_lh   = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_lh;

    Matrix3d Skew_lh_World;
             Skew_lh_World.setZero(3,3);

    Skew_lh_World(0,1) = -World_Posi_lh(2);
    Skew_lh_World(0,2) =  World_Posi_lh(1);
    Skew_lh_World(1,0) =  World_Posi_lh(2);
    Skew_lh_World(1,2) = -World_Posi_lh(0);
    Skew_lh_World(2,0) = -World_Posi_lh(1);
    Skew_lh_World(2,1) =  World_Posi_lh(0);

    // Mapping between base angular velocity and time derivative of base orientation angles.
    // Twist transformation matrices
        
    RootVeloTwistMx_lh_World.block(0,0, 3,3) =  MatrixXd::Identity(3,3);
    RootVeloTwistMx_lh_World.block(0,3, 3,3) = -Skew_lh_World;
    RootVeloTwistMx_lh_World.block(3,3, 3,3) =  MatrixXd::Identity(3,3);


    Lhand_VBase_TskJac_World.block(0,0, 6, Jacobian_Lhand_root.cols()) =  Rot6x6_root_World * Jacobian_Lhand_root;
    Lhand_VBase_TskJac_World.block(0, Jacobian_Lhand_root.cols(), 6,6) =  RootVeloTwistMx_lh_World;     

    return Lhand_VBase_TskJac_World;

}

// Task Jacobian of the left hand associated to the robot root link with left stance leg
MatrixXd Grasping::getRhand_VBase_TskJac_World( MatrixXd wld_Trsf_root,
												iCub::iKin::iKinChain *right_Arm_Chain)
{

	//
	// right hand
    yarp::sig::Vector RightHandPoseInRoot;  // Pose of right hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the right hand frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rh_rt_yarp;  // Jacobian of the right hand wrt. the root frame (as yarp matrix)


    RightHandPoseInRoot = right_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(RightHandPoseInRoot.subVector(3,6));
    Jacobian_rh_rt_yarp = right_Arm_Chain->GeoJacobian();

    // Eigen matrix of left hand Jacobian in root 
    Jacobian_Rhand_root.resize(Jacobian_rh_rt_yarp.rows(), Jacobian_rh_rt_yarp.cols());

    for (int row=0; row<Jacobian_rh_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_rh_rt_yarp.cols(); col++)
        {
            Jacobian_Rhand_root(row, col) = Jacobian_rh_rt_yarp(row, col);
        }
    }

   
    // Rotation Mx of the root frame wrt to a general left foot frame    
    Rot6x6_root_World.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
    Rot6x6_root_World.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);     

    // Mapping between the left hand Veclocity and the left hand joints velocoties and the 
    // the velocity of the virtual base (assumed attached to the root link)
    Rhand_VBase_TskJac_World.resize(6, Jacobian_Rhand_root.cols() + 6);

    // translation left
    Vector3d rt_Posi_rh, World_Posi_rh;
             
    rt_Posi_rh(0) = RightHandPoseInRoot[0];
    rt_Posi_rh(1) = RightHandPoseInRoot[1];
    rt_Posi_rh(2) = RightHandPoseInRoot[2];

    World_Posi_rh   = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_rh;

    Matrix3d Skew_rh_World;
             Skew_rh_World.setZero(3,3);

    Skew_rh_World(0,1) = -World_Posi_rh(2);
    Skew_rh_World(0,2) =  World_Posi_rh(1);
    Skew_rh_World(1,0) =  World_Posi_rh(2);
    Skew_rh_World(1,2) = -World_Posi_rh(0);
    Skew_rh_World(2,0) = -World_Posi_rh(1);
    Skew_rh_World(2,1) =  World_Posi_rh(0);

    // Mapping between base angular velocity and time derivative of base orientation angles.
    // Twist transformation matrices
        
    RootVeloTwistMx_rh_World.block(0,0, 3,3) =  MatrixXd::Identity(3,3);
    RootVeloTwistMx_rh_World.block(0,3, 3,3) = -Skew_rh_World;
    RootVeloTwistMx_rh_World.block(3,3, 3,3) =  MatrixXd::Identity(3,3);


    Rhand_VBase_TskJac_World.block(0,0, 6, Jacobian_Rhand_root.cols()) =  Rot6x6_root_World * Jacobian_Rhand_root;
    Rhand_VBase_TskJac_World.block(0, Jacobian_Rhand_root.cols(), 6,6) =  RootVeloTwistMx_rh_World;


    return Rhand_VBase_TskJac_World;

}



// Task Jacobian of left hand wrt. the stance legs and all expressed in the world frame
MatrixXd Grasping::getLhand_world_TaskJacobian( MatrixXd wld_Trsf_root,
                                                iCub::iKin::iKinChain *left_Arm_Chain, 
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                bool StanceLeg)
{
    
    yarp::sig::Vector LeftHandPoseInRoot;   // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    yarp::sig::Matrix Jacobian_lh_rt_yarp;  // Jacobian of the left hand wrt. the root frame (as yarp matrix)
    // 
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_ll_rt_yarp;  // Jacobian of the left leg wrt. the root frame (as yarp matrix)
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rl_rt_yarp;  // Jacobian of the right leg wrt. the root frame (as yarp matrix)

    

    LeftHandPoseInRoot  = left_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(LeftHandPoseInRoot.subVector(3,6));
    Jacobian_lh_rt_yarp = left_Arm_Chain->GeoJacobian();

    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));
    Jacobian_ll_rt_yarp = LeftLegChain->GeoJacobian();

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));
    Jacobian_rl_rt_yarp = RightLegChain->GeoJacobian();

    // Eigen matrix of left hand Jacobian in root 
    Jacobian_Lhand_root.resize(Jacobian_lh_rt_yarp.rows(), Jacobian_lh_rt_yarp.cols());

    for (int row=0; row<Jacobian_lh_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_lh_rt_yarp.cols(); col++)
        {
            Jacobian_Lhand_root(row, col) = Jacobian_lh_rt_yarp(row, col);
        }
    }

    // Eigen matrix of left leg Jacobian in root 
    Jacobian_Lleg_root.resize(Jacobian_ll_rt_yarp.rows(), Jacobian_ll_rt_yarp.cols());

    for (int row=0; row<Jacobian_ll_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_ll_rt_yarp.cols(); col++)
        {
            Jacobian_Lleg_root(row, col) = Jacobian_ll_rt_yarp(row, col);
        }
    }

    // Eigen matrix of right leg Jacobian in root 
    Jacobian_Rleg_root.resize(Jacobian_rl_rt_yarp.rows(), Jacobian_rl_rt_yarp.cols());

    for (int row=0; row<Jacobian_rl_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_rl_rt_yarp.cols(); col++)
        {
            Jacobian_Rleg_root(row, col) = Jacobian_rl_rt_yarp(row, col);
        }
    }

    // rotation Mx of the root frame wrt. the world frame
    Rot6x6_root_World.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
    Rot6x6_root_World.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);

    // rotation Mx of the root frame wrt. the world frame
    MatrixXd RootVeloTwistMx_lh_leg_World;
             RootVeloTwistMx_lh_leg_World = MatrixXd::Identity(6,6);

        // translation left
        // ~~~~~~~~~~~~~~~~
        Vector3d rt_Posi_lh, World_Posi_lh;
             
        rt_Posi_lh(0) = LeftHandPoseInRoot[0];
        rt_Posi_lh(1) = LeftHandPoseInRoot[1];
        rt_Posi_lh(2) = LeftHandPoseInRoot[2];

        World_Posi_lh = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_lh;

        Matrix3d Skew_lh_World;
                 Skew_lh_World.setZero(3,3);

        Skew_lh_World(0,1) = -World_Posi_lh(2);
        Skew_lh_World(0,2) =  World_Posi_lh(1);
        Skew_lh_World(1,0) =  World_Posi_lh(2);
        Skew_lh_World(1,2) = -World_Posi_lh(0);
        Skew_lh_World(2,0) = -World_Posi_lh(1);
        Skew_lh_World(2,1) =  World_Posi_lh(0);

        // Mapping between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices

        RootVeloTwistMx_lh_leg_World.block(0,3, 3,3) = -Skew_lh_World;
                    

        // Mapping between the left hand Veclocity and the left hand joints velocoties and 
        // 1) the velocity of the virtual base (assumed attached to the root link)
        Lhand_virtBase_TaskJacobian.resize(6, Jacobian_Lhand_root.cols() + 6);
        Lhand_virtBase_TaskJacobian.block(0,0, 6, Jacobian_Lhand_root.cols()) =  Rot6x6_root_World * Jacobian_Lhand_root;
        Lhand_virtBase_TaskJacobian.block(0, Jacobian_Lhand_root.cols(), 6,6) =  RootVeloTwistMx_lh_leg_World;

        // 2) the velocity of the stance leg
        Lhand_StanceLeg_TaskJacobian.resize(6, Jacobian_Lhand_root.cols() + Jacobian_Lleg_root.cols());

        

    if (StanceLeg) // left stance leg
    {
        
        // translation left leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_ll, World_Posi_ll;
         
        rt_Posi_ll(0) = LeftFootPoseInRoot[0];
        rt_Posi_ll(1) = LeftFootPoseInRoot[1];
        rt_Posi_ll(2) = LeftFootPoseInRoot[2];

        World_Posi_ll  = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_ll;

        Matrix3d Skew_ll_World;
                 Skew_ll_World.setZero(3,3);

        Skew_ll_World(0,1) = -World_Posi_ll(2);
        Skew_ll_World(0,2) =  World_Posi_ll(1);
        Skew_ll_World(1,0) =  World_Posi_ll(2);
        Skew_ll_World(1,2) = -World_Posi_ll(0);
        Skew_ll_World(2,0) = -World_Posi_ll(1);
        Skew_ll_World(2,1) =  World_Posi_ll(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_LeftStanceLeg_Jacobian.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
        Root_LeftStanceLeg_Jacobian.block(0,3, 3,3) = Skew_ll_World * wld_Trsf_root.block(0,0, 3,3);
        Root_LeftStanceLeg_Jacobian.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);

        //
        //
        Lhand_StanceLeg_TaskJacobian.block(0,0, 6, Jacobian_Lhand_root.cols()) =  Rot6x6_root_World * Jacobian_Lhand_root;
        Lhand_StanceLeg_TaskJacobian.block(0, Jacobian_Lhand_root.cols(), 6, Jacobian_Lleg_root.cols()) = -RootVeloTwistMx_lh_leg_World * Root_LeftStanceLeg_Jacobian * Jacobian_Lleg_root;  


    }
    else
    {
        
        // translation right leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_rl, World_Posi_rl;
         
        rt_Posi_rl(0) = RightFootPoseInRoot[0];
        rt_Posi_rl(1) = RightFootPoseInRoot[1];
        rt_Posi_rl(2) = RightFootPoseInRoot[2];

        World_Posi_rl   = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_rl;

        Matrix3d Skew_rl_World;
                 Skew_rl_World.setZero(3,3);

        Skew_rl_World(0,1) = -World_Posi_rl(2);
        Skew_rl_World(0,2) =  World_Posi_rl(1);
        Skew_rl_World(1,0) =  World_Posi_rl(2);
        Skew_rl_World(1,2) = -World_Posi_rl(0);
        Skew_rl_World(2,0) = -World_Posi_rl(1);
        Skew_rl_World(2,1) =  World_Posi_rl(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_RightStanceLeg_Jacobian.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
        Root_RightStanceLeg_Jacobian.block(0,3, 3,3) = Skew_rl_World * wld_Trsf_root.block(0,0, 3,3);
        Root_RightStanceLeg_Jacobian.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);

        //
        //
        Lhand_StanceLeg_TaskJacobian.block(0,0, 6, Jacobian_Lhand_root.cols()) =  Rot6x6_root_World * Jacobian_Lhand_root;
        Lhand_StanceLeg_TaskJacobian.block(0, Jacobian_Lhand_root.cols(), 6, Jacobian_Rleg_root.cols()) = -RootVeloTwistMx_lh_leg_World * Root_RightStanceLeg_Jacobian * Jacobian_Rleg_root;

    }
      

    return Lhand_StanceLeg_TaskJacobian;
}


//
// Task jacobian of right hand wrt. the stance legs and all expressed in the world frame
MatrixXd Grasping::getRhand_world_TaskJacobian( MatrixXd wld_Trsf_root,
                                                iCub::iKin::iKinChain *right_Arm_Chain, 
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                bool StanceLeg)
{
    yarp::sig::Vector RightHandPoseInRoot;  // Pose of left hand frame wrt. the root frame
    yarp::sig::Matrix Rot_HandInRoot;       // Rotation of the left hand frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rh_rt_yarp;  // Jacobian of the left hand wrt. the root frame (as yarp matrix)
    // 
    yarp::sig::Vector LeftFootPoseInRoot;   // Pose of left foot frame wrt. the root frame
    yarp::sig::Matrix Rot_LeftFootInRoot;   // Rotation of the left foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_ll_rt_yarp;  // Jacobian of the left leg wrt. the root frame (as yarp matrix)
    //
    yarp::sig::Vector RightFootPoseInRoot;  // Pose of right foot frame wrt. the root frame
    yarp::sig::Matrix Rot_RightFootInRoot;  // Rotation of the right foot frame wrt. the root frame
    yarp::sig::Matrix Jacobian_rl_rt_yarp;  // Jacobian of the right leg wrt. the root frame (as yarp matrix)

    

    RightHandPoseInRoot = right_Arm_Chain->EndEffPose();
    Rot_HandInRoot      = yarp::math::axis2dcm(RightHandPoseInRoot.subVector(3,6));
    Jacobian_rh_rt_yarp = right_Arm_Chain->GeoJacobian();

    LeftFootPoseInRoot  = LeftLegChain->EndEffPose();
    Rot_LeftFootInRoot  = yarp::math::axis2dcm(LeftFootPoseInRoot.subVector(3,6));
    Jacobian_ll_rt_yarp = LeftLegChain->GeoJacobian();

    RightFootPoseInRoot = RightLegChain->EndEffPose();
    Rot_RightFootInRoot = yarp::math::axis2dcm(RightFootPoseInRoot.subVector(3,6));
    Jacobian_rl_rt_yarp = RightLegChain->GeoJacobian();

    // Eigen matrix of left hand Jacobian in root 
    Jacobian_Rhand_root.resize(Jacobian_rh_rt_yarp.rows(), Jacobian_rh_rt_yarp.cols());

    for (int row=0; row<Jacobian_rh_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_rh_rt_yarp.cols(); col++)
        {
            Jacobian_Rhand_root(row, col) = Jacobian_rh_rt_yarp(row, col);
        }
    }

    // Eigen matrix of left leg Jacobian in root 
    Jacobian_Lleg_root.resize(Jacobian_ll_rt_yarp.rows(), Jacobian_ll_rt_yarp.cols());

    for (int row=0; row<Jacobian_ll_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_ll_rt_yarp.cols(); col++)
        {
            Jacobian_Lleg_root(row, col) = Jacobian_ll_rt_yarp(row, col);
        }
    }

    // Eigen matrix of right leg Jacobian in root 
    Jacobian_Rleg_root.resize(Jacobian_rl_rt_yarp.rows(), Jacobian_rl_rt_yarp.cols());

    for (int row=0; row<Jacobian_rl_rt_yarp.rows(); row++)
    {
        for(int col=0; col<Jacobian_rl_rt_yarp.cols(); col++)
        {
            Jacobian_Rleg_root(row, col) = Jacobian_rl_rt_yarp(row, col);
        }
    }

   // rotation Mx of the root frame wrt. the world frame
    Rot6x6_root_World.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
    Rot6x6_root_World.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);

    // rotation Mx of the root frame wrt. the world frame
    MatrixXd RootVeloTwistMx_rh_leg_World;
             RootVeloTwistMx_rh_leg_World = MatrixXd::Identity(6,6);

        // translation right
        // ~~~~~~~~~~~~~~~~
        Vector3d rt_Posi_rh, World_Posi_rh;
             
        rt_Posi_rh(0) = RightHandPoseInRoot[0];
        rt_Posi_rh(1) = RightHandPoseInRoot[1];
        rt_Posi_rh(2) = RightHandPoseInRoot[2];

        World_Posi_rh = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_rh;

        Matrix3d Skew_rh_World;
                 Skew_rh_World.setZero(3,3);

        Skew_rh_World(0,1) = -World_Posi_rh(2);
        Skew_rh_World(0,2) =  World_Posi_rh(1);
        Skew_rh_World(1,0) =  World_Posi_rh(2);
        Skew_rh_World(1,2) = -World_Posi_rh(0);
        Skew_rh_World(2,0) = -World_Posi_rh(1);
        Skew_rh_World(2,1) =  World_Posi_rh(0);

        // Mapping between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices

        RootVeloTwistMx_rh_leg_World.block(0,3, 3,3) = -Skew_rh_World;
                    

        // Mapping between the left hand Veclocity and the left hand joints velocoties and 
        // 1) the velocity of the virtual base (assumed attached to the root link)
        Rhand_virtBase_TaskJacobian.resize(6, Jacobian_Rhand_root.cols() + 6);

        Rhand_virtBase_TaskJacobian.block(0,0, 6, Jacobian_Rhand_root.cols()) =  Rot6x6_root_World * Jacobian_Rhand_root;
        Rhand_virtBase_TaskJacobian.block(0, Jacobian_Rhand_root.cols(), 6,6) =  RootVeloTwistMx_rh_leg_World;

        // 2) the velocity of the stance leg
        Rhand_StanceLeg_TaskJacobian.resize(6, Jacobian_Rhand_root.cols() + Jacobian_Lleg_root.cols());


    if (StanceLeg) // left stance leg
    {
        
        // translation left leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_ll, World_Posi_ll;
         
        rt_Posi_ll(0) = LeftFootPoseInRoot[0];
        rt_Posi_ll(1) = LeftFootPoseInRoot[1];
        rt_Posi_ll(2) = LeftFootPoseInRoot[2];

        World_Posi_ll  = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_ll;

        Matrix3d Skew_ll_World;
                 Skew_ll_World.setZero(3,3);

        Skew_ll_World(0,1) = -World_Posi_ll(2);
        Skew_ll_World(0,2) =  World_Posi_ll(1);
        Skew_ll_World(1,0) =  World_Posi_ll(2);
        Skew_ll_World(1,2) = -World_Posi_ll(0);
        Skew_ll_World(2,0) = -World_Posi_ll(1);
        Skew_ll_World(2,1) =  World_Posi_ll(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_LeftStanceLeg_Jacobian.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
        Root_LeftStanceLeg_Jacobian.block(0,3, 3,3) = Skew_ll_World * wld_Trsf_root.block(0,0, 3,3);
        Root_LeftStanceLeg_Jacobian.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);

        //
        //
        Rhand_StanceLeg_TaskJacobian.block(0,0, 6, Jacobian_Rhand_root.cols()) =  Rot6x6_root_World * Jacobian_Rhand_root;
        Rhand_StanceLeg_TaskJacobian.block(0, Jacobian_Rhand_root.cols(), 6, Jacobian_Lleg_root.cols()) = -RootVeloTwistMx_rh_leg_World * Root_LeftStanceLeg_Jacobian * Jacobian_Lleg_root;     


    }
    else
    {

        // translation right leg
        // ~~~~~~~~~~~~~~~~~~~~~

        Vector3d rt_Posi_rl, World_Posi_rl;
         
        rt_Posi_rl(0) = RightFootPoseInRoot[0];
        rt_Posi_rl(1) = RightFootPoseInRoot[1];
        rt_Posi_rl(2) = RightFootPoseInRoot[2];

        World_Posi_rl   = wld_Trsf_root.block(0,0, 3,3) * rt_Posi_rl;

        Matrix3d Skew_rl_World;
             Skew_rl_World.setZero(3,3);

        Skew_rl_World(0,1) = -World_Posi_rl(2);
        Skew_rl_World(0,2) =  World_Posi_rl(1);
        Skew_rl_World(1,0) =  World_Posi_rl(2);
        Skew_rl_World(1,2) = -World_Posi_rl(0);
        Skew_rl_World(2,0) = -World_Posi_rl(1);
        Skew_rl_World(2,1) =  World_Posi_rl(0);

        // Map between base angular velocity and time derivative of base orientation angles.
        // Twist transformation matrices
        
        Root_RightStanceLeg_Jacobian.block(0,0, 3,3) = wld_Trsf_root.block(0,0, 3,3);
        Root_RightStanceLeg_Jacobian.block(0,3, 3,3) = Skew_rl_World * wld_Trsf_root.block(0,0, 3,3);
        Root_RightStanceLeg_Jacobian.block(3,3, 3,3) = wld_Trsf_root.block(0,0, 3,3);

        //
        //
        Rhand_StanceLeg_TaskJacobian.block(0,0, 6, Jacobian_Rhand_root.cols()) =  Rot6x6_root_World * Jacobian_Rhand_root;
        Rhand_StanceLeg_TaskJacobian.block(0, Jacobian_Rhand_root.cols(), 6, Jacobian_Rleg_root.cols()) = -RootVeloTwistMx_rh_leg_World * Root_RightStanceLeg_Jacobian * Jacobian_Rleg_root;


    }
      

    return Rhand_StanceLeg_TaskJacobian;
}


//  Grasp jacobian
// ~~~~~~~~~~~~~~~~~
// 
// Grasping Jacobian of the left hand wrt. the General stance leg
MatrixXd Grasping::getGraspJacobianLhand_World( MatrixXd wld_Trsf_root,
												MatrixXd dl_H_cl,
												iCub::iKin::iKinChain *left_Arm_Chain, 
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                bool StanceLeg)
{
    
    MatrixXd L_eta_lhand;
    MatrixXd W_World_lhand;
    MatrixXd J_task_lhand_World;

    L_eta_lhand = Grasping::getInteractionMxForAxisAngle(dl_H_cl);

    W_World_lhand = Grasping::getLhand_World_VeloTwistMx(	wld_Trsf_root,
										  					left_Arm_Chain);

    J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                          						left_Arm_Chain, 
                                          						LeftLegChain,
                                          						RightLegChain,
                                          						StanceLeg);

    GraspJacobianLhand_World = L_eta_lhand * W_World_lhand * J_task_lhand_World;

    return GraspJacobianLhand_World; 

}

// Grasping Jacobian of the right hand wrt. the General stance leg
MatrixXd Grasping::getGraspJacobianRhand_World( MatrixXd wld_Trsf_root,
												MatrixXd dr_H_cr,
												iCub::iKin::iKinChain *right_Arm_Chain, 
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                bool StanceLeg)
{
    
    MatrixXd L_eta_rhand;
    MatrixXd W_World_rhand;
    MatrixXd J_task_rhand_World;

    L_eta_rhand = Grasping::getInteractionMxForAxisAngle(dr_H_cr);

    W_World_rhand = Grasping::getRhand_World_VeloTwistMx(	wld_Trsf_root,
										  					right_Arm_Chain);

    J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                          						right_Arm_Chain, 
                                          						LeftLegChain,
                                          						RightLegChain,
                                          						StanceLeg);

    GraspJacobianRhand_World = L_eta_rhand * W_World_rhand * J_task_rhand_World;

    return GraspJacobianRhand_World; 

}


// left hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::get_lh_GraspJtsVelocity_World(   MatrixXd wld_Trsf_root,
													MatrixXd dl_H_cl,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Lhand_leg_World;
    MatrixXd J_grasp_Lhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd q_dot_Lhand;

    MatrixXd PsdInv_grasp_Jac_Lhand;
    MatrixXd NullSpace_grasp_Jac_Lhand;

    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getLeftHand_JointsLimitsGradient(left_Arm_Chain, rho);

    e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);

    
    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        J_grap_Lhand_leg_World = Grasping::getGraspJacobianLhand_World(	wld_Trsf_root,
																		dl_H_cl,
																		left_Arm_Chain, 
	                                                					LeftLegChain,
	                                                					RightLegChain,
	                                                					StanceLeg);

    
        J_grasp_Lhand     		= J_grap_Lhand_leg_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_grap_Lhand_leg_World.block(0, n_lh, 6, n_ll);

        PsdInv_grasp_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_grasp_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_grasp_Jac_Lhand * J_grasp_Lhand;

        // q_dot_Lhand = PsdInv_grasp_Jac_Lhand * (-1.0 *graspGain_lh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_lleg) 
        //             - grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;
        q_dot_Lhand = PsdInv_grasp_Jac_Lhand * (-3.0 * e_grasp - J_grasp_stanceleg_World * q_dot_lleg) 
                    - grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;

    }
    else
    {
        J_grap_Lhand_leg_World = Grasping::getGraspJacobianLhand_World(	wld_Trsf_root,
																		dl_H_cl,
																		left_Arm_Chain, 
	                                                					LeftLegChain,
	                                                					RightLegChain,
	                                                					StanceLeg);

    
        J_grasp_Lhand     		= J_grap_Lhand_leg_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_grap_Lhand_leg_World.block(0, n_lh, 6, n_ll);

        PsdInv_grasp_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_grasp_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_grasp_Jac_Lhand * J_grasp_Lhand;

        // q_dot_Lhand = PsdInv_grasp_Jac_Lhand * (-1.0 *graspGain_lh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_rleg)
        //             - grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;
        q_dot_Lhand = PsdInv_grasp_Jac_Lhand * (-3.0 * e_grasp - J_grasp_stanceleg_World * q_dot_rleg)
                    - grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;

    }

    qDot_Grasping_left_hand = q_dot_Lhand;

    return q_dot_Lhand;

}


// right hand joint velocities for the grasping task  wrt. the world frame
VectorXd Grasping::get_rh_GraspJtsVelocity_World(   MatrixXd wld_Trsf_root,
													MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    VectorXd e_grasp;
    MatrixXd J_grap_Rhand_leg_World;
    MatrixXd J_grasp_Rhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd q_dot_Rhand;

    MatrixXd PsdInv_grasp_Jac_Rhand;
    MatrixXd NullSpace_grasp_Jac_Rhand;

    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.000;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getRightHand_JointsLimitsGradient(right_Arm_Chain, rho);


    e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);

    VectorXd e_grasp_pos;
    VectorXd e_grasp_ori;

    e_grasp_pos = e_grasp.segment(0,3);
    e_grasp_ori = e_grasp.segment(3,3);

    double Threshold_pos = 0.05;
    double Threshold_ori = 0.01;

    double epsilon_pos;
    double lambda_pos; // = (e_grasp_ori.transpose() * e_grasp_ori)/(Threshold_ori * Threshold_ori);
    lambda_pos = (e_grasp_ori(0)*e_grasp_ori(0) + e_grasp_ori(1)*e_grasp_ori(1) + e_grasp_ori(2)*e_grasp_ori(2))/(Threshold_ori * Threshold_ori);
    epsilon_pos = exp(-lambda_pos);
    
    double epsilon_ori;
    double lambda_ori; // = (e_grasp_pos.transpose() * e_grasp_pos)/(Threshold_pos * Threshold_pos);
    lambda_ori = (e_grasp_pos(0)*e_grasp_pos(0) + e_grasp_pos(1)*e_grasp_pos(1) + e_grasp_pos(2)*e_grasp_pos(2))/(Threshold_pos * Threshold_pos);
    epsilon_ori = exp(-lambda_ori);


    MatrixXd Comb_Projector_pos;
    MatrixXd Class_Projector_pos;
    MatrixXd New_Projector_pos;

    MatrixXd Comb_Projector_ori;
    MatrixXd Class_Projector_ori;
    MatrixXd New_Projector_ori;

    MatrixXd J_grasp_Rhand_pos;
    MatrixXd J_grasp_Rhand_ori;
    MatrixXd J_grasp_stanceleg_World_pos;
    MatrixXd J_grasp_stanceleg_World_ori;
    //
    MatrixXd PsdInv_J_grasp_Rhand_pos;
    MatrixXd PsdInv_J_grasp_Rhand_ori;


    

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        J_grap_Rhand_leg_World = Grasping::getGraspJacobianRhand_World( wld_Trsf_root,
															 			dr_H_cr,
															 			right_Arm_Chain, 
				                                                		LeftLegChain,
				                                                		RightLegChain,
				                                                		StanceLeg);
    
    
        J_grasp_Rhand     		= J_grap_Rhand_leg_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_grap_Rhand_leg_World.block(0, n_rh, 6, n_ll);

        // PsdInv_grasp_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        // NullSpace_grasp_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_grasp_Jac_Rhand * J_grasp_Rhand;

        // q_dot_Rhand = PsdInv_grasp_Jac_Rhand * (-1.0 *graspGain_rh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_lleg)
        //             - grad_gain * NullSpace_grasp_Jac_Rhand * joints_limit_gradient;

        // with partition approach
        J_grasp_Rhand_pos = J_grasp_Rhand.block(0, 0, 3, n_rh);   
        J_grasp_stanceleg_World_pos = J_grasp_stanceleg_World.block(0, 0, 3, n_ll);

        PsdInv_J_grasp_Rhand_pos  = MxPsdInv.pseudoInverse(J_grasp_Rhand_pos);


        Class_Projector_pos  = MatrixXd::Identity(n_rh, n_rh) - PsdInv_J_grasp_Rhand_pos * J_grasp_Rhand_pos;
        New_Projector_pos    = MatrixXd::Identity(n_rh, n_rh) - (1./(e_grasp_pos.transpose() * J_grasp_Rhand_pos * J_grasp_Rhand_pos.transpose() * e_grasp_pos))*(J_grasp_Rhand_pos.transpose() * e_grasp_pos * e_grasp_pos.transpose() * J_grasp_Rhand_pos);
        Comb_Projector_pos   = 1.0*((1. - epsilon_pos) * New_Projector_pos + epsilon_pos * Class_Projector_pos); 
        
        // Orientation
        J_grasp_Rhand_ori = J_grasp_Rhand.block(3, 0, 3, n_rh);
        J_grasp_stanceleg_World_ori = J_grasp_stanceleg_World.block(3, 0, 3, n_ll);

        PsdInv_J_grasp_Rhand_ori  = MxPsdInv.pseudoInverse(J_grasp_Rhand_ori);

        Class_Projector_ori  = MatrixXd::Identity(n_rh, n_rh) - PsdInv_J_grasp_Rhand_ori * J_grasp_Rhand_ori;
        New_Projector_ori    = MatrixXd::Identity(n_rh, n_rh) - (1/(e_grasp_ori.transpose() * J_grasp_Rhand_ori * J_grasp_Rhand_ori.transpose() * e_grasp_ori))*(J_grasp_Rhand_ori.transpose() * e_grasp_ori * e_grasp_ori.transpose() * J_grasp_Rhand_ori);
        Comb_Projector_ori   = (1. - epsilon_ori) * New_Projector_ori + epsilon_ori * Class_Projector_ori;


        // q_dot_Rhand = PsdInv_J_grasp_Rhand_pos * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_pos - J_grasp_stanceleg_World_pos * q_dot_lleg)
        //             + Comb_Projector_pos * PsdInv_J_grasp_Rhand_ori * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_ori - J_grasp_stanceleg_World_ori * q_dot_lleg)
        //             - grad_gain * Comb_Projector_pos * joints_limit_gradient; 

        q_dot_Rhand = PsdInv_J_grasp_Rhand_pos * (-0.5 * e_grasp_pos - J_grasp_stanceleg_World_pos * q_dot_lleg)
                    + Comb_Projector_pos * PsdInv_J_grasp_Rhand_ori * (-0.5 * e_grasp_ori - J_grasp_stanceleg_World_ori * q_dot_lleg)
                    - grad_gain * Comb_Projector_pos * joints_limit_gradient;     


        // q_dot_Rhand = PsdInv_J_grasp_Rhand_ori * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_ori - J_grasp_stanceleg_World_ori * q_dot_lleg)
        //             + Comb_Projector_ori * PsdInv_J_grasp_Rhand_pos * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_pos - J_grasp_stanceleg_World_pos * q_dot_lleg)
        //             - grad_gain * Comb_Projector_ori * joints_limit_gradient;

    }
    else
    {
        J_grap_Rhand_leg_World = Grasping::getGraspJacobianRhand_World( wld_Trsf_root,
															 			dr_H_cr,
															 			right_Arm_Chain, 
				                                                		LeftLegChain,
				                                                		RightLegChain,
				                                                		StanceLeg);
    
    
        J_grasp_Rhand     		= J_grap_Rhand_leg_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_grap_Rhand_leg_World.block(0, n_rh, 6, n_ll);

        // PsdInv_grasp_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        // NullSpace_grasp_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_grasp_Jac_Rhand * J_grasp_Rhand;

        // q_dot_Rhand = PsdInv_grasp_Jac_Rhand * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp.segment(3,3) - J_grasp_stanceleg_World * q_dot_rleg)
        //             - grad_gain * NullSpace_grasp_Jac_Rhand * joints_limit_gradient;

        // with partition approach
        J_grasp_Rhand_pos = J_grasp_Rhand.block(0, 0, 3, n_rh);   
        J_grasp_stanceleg_World_pos = J_grasp_stanceleg_World.block(0, 0, 3, n_ll);

        PsdInv_J_grasp_Rhand_pos  = MxPsdInv.pseudoInverse(J_grasp_Rhand_pos);

        Class_Projector_pos  = MatrixXd::Identity(n_rh, n_rh) - PsdInv_J_grasp_Rhand_pos * J_grasp_Rhand_pos;
        New_Projector_pos    = MatrixXd::Identity(n_rh, n_rh) - (1./(e_grasp_pos.transpose() * J_grasp_Rhand_pos * J_grasp_Rhand_pos.transpose() * e_grasp_pos))*(J_grasp_Rhand_pos.transpose() * e_grasp_pos * e_grasp_pos.transpose() * J_grasp_Rhand_pos);
        Comb_Projector_pos   = ((1. - epsilon_pos) * New_Projector_pos + epsilon_pos * Class_Projector_pos); 

        // Orientation
        J_grasp_Rhand_ori = J_grasp_Rhand.block(3, 0, 3, n_rh);
        J_grasp_stanceleg_World_ori = J_grasp_stanceleg_World.block(3, 0, 3, n_ll);

        PsdInv_J_grasp_Rhand_ori  = MxPsdInv.pseudoInverse(J_grasp_Rhand_ori);

        Class_Projector_ori  = MatrixXd::Identity(n_rh, n_rh) - PsdInv_J_grasp_Rhand_ori * J_grasp_Rhand_ori;
        New_Projector_ori    = MatrixXd::Identity(n_rh, n_rh) - (1./(e_grasp_ori.transpose() * J_grasp_Rhand_ori * J_grasp_Rhand_ori.transpose() * e_grasp_ori))*(J_grasp_Rhand_ori.transpose() * e_grasp_ori * e_grasp_ori.transpose() * J_grasp_Rhand_ori);
        Comb_Projector_ori   = 1.0*(1 - epsilon_ori) * New_Projector_ori + epsilon_ori * Class_Projector_ori;

        q_dot_Rhand = PsdInv_J_grasp_Rhand_pos * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_pos - J_grasp_stanceleg_World_pos * q_dot_rleg)
                    + Comb_Projector_pos * PsdInv_J_grasp_Rhand_ori * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_ori - J_grasp_stanceleg_World_ori * q_dot_rleg)
                    - grad_gain * Comb_Projector_pos * joints_limit_gradient;     
      

        // q_dot_Rhand = PsdInv_J_grasp_Rhand_ori * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_ori - J_grasp_stanceleg_World_ori * q_dot_rleg)
        //             + Comb_Projector_ori * PsdInv_J_grasp_Rhand_pos * (-1.0 *(graspGain_rh.segment(3,3)).asDiagonal() * e_grasp_pos - J_grasp_stanceleg_World_pos * q_dot_rleg)
        //             - grad_gain * Comb_Projector_ori * joints_limit_gradient;

    }

    qDot_Grasping_right_hand = q_dot_Rhand;

    return q_dot_Rhand;
}

// bimanual joint velocities for the grasping task  wrt. the world frame
void Grasping::get_bimanual_GraspJtsVelocity_World( MatrixXd wld_Trsf_root,
													MatrixXd dl_H_cl,
                                                    MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *right_Arm_Chain, 
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg)
{
    VectorXd e_grasp_lh, 
             e_grasp_rh,
             e_bimanual_grasp;

    MatrixXd J_grap_Lhand_leg_World,
             J_grap_Rhand_leg_World;

    MatrixXd J_grasp_Lhand, 
             J_grasp_Rhand,
             J_grasp_bimanual;

    MatrixXd J_Lgrasp_stanceleg_world, 
             J_Rgrasp_stanceleg_world;

    VectorXd graspGain_bimanual;

    VectorXd q_dot_LR_hands;


    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();

    q_dot_LR_hands.resize(2*n_lh);
    q_dot_LR_hands.setZero(2*n_lh);

    J_grasp_bimanual.resize(2*6, 2*n_lh);
    J_grasp_bimanual.setZero(2*6, 2*n_lh);

    graspGain_bimanual.resize(graspGain_lh.rows()+graspGain_rh.rows());
    graspGain_bimanual.segment(0, graspGain_lh.rows()) = graspGain_lh;
    graspGain_bimanual.segment(graspGain_lh.rows(), graspGain_rh.rows()) = graspGain_rh;


    e_grasp_lh = Grasping::getPoseError_d_H_c(dl_H_cl);
    e_grasp_rh = Grasping::getPoseError_d_H_c(dr_H_cr);

    e_bimanual_grasp.resize(e_grasp_lh.rows()+ e_grasp_rh.rows());
    


    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        
        J_grap_Lhand_leg_World = Grasping::getGraspJacobianLhand_World(	wld_Trsf_root,
																		dl_H_cl,
																		left_Arm_Chain, 
		                                                				LeftLegChain,
		                                                				RightLegChain,
		                                                				StanceLeg);

        J_grasp_Lhand      		 = J_grap_Lhand_leg_World.block(0, 0, 6, n_lh);
        J_Lgrasp_stanceleg_world = J_grap_Lhand_leg_World.block(0, n_lh, 6, n_ll);

        J_grap_Rhand_leg_World = Grasping::getGraspJacobianRhand_World(	wld_Trsf_root,
																 		dr_H_cr,
																 		right_Arm_Chain, 
					                                                	LeftLegChain,
					                                                	RightLegChain,
					                                                	StanceLeg);
        
        J_grasp_Rhand      		 = J_grap_Rhand_leg_World.block(0, 0, 6, n_lh);
        J_Rgrasp_stanceleg_world = J_grap_Rhand_leg_World.block(0, n_lh, 6, n_ll);

        // bimanual grasp jacobian
        J_grasp_bimanual.block(0,0, 6, n_lh)    = J_grasp_Lhand;
        J_grasp_bimanual.block(6,n_lh, 6, n_lh) = J_grasp_Rhand;

        e_bimanual_grasp.segment(0, e_grasp_lh.rows()) = -1.0 * graspGain_lh.asDiagonal() * e_grasp_lh - J_Lgrasp_stanceleg_world * q_dot_lleg;
        e_bimanual_grasp.segment(e_grasp_lh.rows(), e_grasp_rh.rows()) = -1.0 *graspGain_rh.asDiagonal() * e_grasp_rh - J_Rgrasp_stanceleg_world * q_dot_lleg;

        q_dot_LR_hands = MxPsdInv.pseudoInverse(J_grasp_bimanual) * e_bimanual_grasp;


    }
    else
    {
        
        J_grap_Lhand_leg_World = Grasping::getGraspJacobianLhand_World(	wld_Trsf_root,
																		dl_H_cl,
																		left_Arm_Chain, 
		                                                				LeftLegChain,
		                                                				RightLegChain,
		                                                				StanceLeg);

        J_grasp_Lhand      		 = J_grap_Lhand_leg_World.block(0, 0, 6, n_lh);
        J_Lgrasp_stanceleg_world = J_grap_Lhand_leg_World.block(0, n_lh, 6, n_ll);

        J_grap_Rhand_leg_World = Grasping::getGraspJacobianRhand_World(	wld_Trsf_root,
																 		dr_H_cr,
																 		right_Arm_Chain, 
					                                                	LeftLegChain,
					                                                	RightLegChain,
					                                                	StanceLeg);
        
        J_grasp_Rhand      		 = J_grap_Rhand_leg_World.block(0, 0, 6, n_lh);
        J_Rgrasp_stanceleg_world = J_grap_Rhand_leg_World.block(0, n_lh, 6, n_ll);

        // bimanual grasp jacobian
        J_grasp_bimanual.block(0,0, 6, n_lh)    = J_grasp_Lhand;
        J_grasp_bimanual.block(6,n_lh, 6, n_lh) = J_grasp_Rhand;

        e_bimanual_grasp.segment(0, e_grasp_lh.rows()) = -1.0 * graspGain_lh.asDiagonal() * e_grasp_lh - J_Lgrasp_stanceleg_world * q_dot_rleg;
        e_bimanual_grasp.segment(e_grasp_lh.rows(), e_grasp_rh.rows()) = -1.0 *graspGain_rh.asDiagonal() * e_grasp_rh - J_Rgrasp_stanceleg_world * q_dot_rleg;

        q_dot_LR_hands = MxPsdInv.pseudoInverse(J_grasp_bimanual) * e_bimanual_grasp;

    }

    qDot_Grasping_bimanual   = q_dot_LR_hands;
    qDot_Grasping_left_hand  = q_dot_LR_hands.segment(0, n_lh);
    qDot_Grasping_right_hand = q_dot_LR_hands.segment(n_lh, n_lh);

}

//

// left hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::get_lh_GraspJtsVelocity_World2(  MatrixXd wld_Trsf_root,
                                                    MatrixXd dl_H_cl,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Lhand_leg_World;
    MatrixXd J_grasp_Lhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd q_dot_Lhand;

    MatrixXd L_eta_lhand;
    MatrixXd W_World_lhand;

    e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);
    // e_grasp(0) = 0.0;
    // e_grasp(1) = 0.1;
    // e_grasp(2) = 0.0;
    // e_grasp(3) = 0.0;
    // e_grasp(4) = 0.0;
    // e_grasp(5) = 0.0;

    L_eta_lhand = Grasping::getInteractionMxForAxisAngle(dl_H_cl);

    W_World_lhand = Grasping::getLhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            left_Arm_Chain);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;
    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_lhand_in_world;
    MatrixXd Hand_interactionMx_world(6,6);
    // Hand_interactionMx_world = L_eta_lhand * W_World_lhand;
    //Velo_lhand_in_world = -1.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * graspGain_lh.asDiagonal() * e_grasp;

    // Hand_interactionMx_world = L_eta_lhand; //.block(0,0, 3,6); // * W_World_lhand;
    // //Velo_lhand_in_world = -3.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * (graspGain_lh.segment(0,3)).asDiagonal() * e_grasp.segment(0,3);
    // Velo_lhand_in_world = -3.0* graspGain_lh(0)*W_World_lhand.transpose()* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * e_grasp; //.segment(0,3);
    
    Hand_interactionMx_world.setZero();
    Hand_interactionMx_world.block<3,3>(0,0) = dl_H_cl.block<3,3>(0,0).transpose();
    Hand_interactionMx_world.block<3,3>(3,3) = MatrixXd::Identity(3,3);
    Velo_lhand_in_world = -3.0* graspGain_lh(0)*W_World_lhand.transpose()* Hand_interactionMx_world * e_grasp;
   
    MatrixXd PsdInv_Jac_Lhand;
    MatrixXd NullSpace_Jac_Lhand;

    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getLeftHand_JointsLimitsGradient(left_Arm_Chain, rho);

    MatrixXd J_task_lhand_World;
    

    if(StanceLeg)
    {
        J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        //PsdInv_Jac_Lhand    = J_grasp_Lhand.transpose() * (J_grasp_Lhand*J_grasp_Lhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Lhand);
        PsdInv_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_Jac_Lhand * J_grasp_Lhand;

        q_dot_Lhand = PsdInv_Jac_Lhand * (Velo_lhand_in_world - J_grasp_stanceleg_World * q_dot_lleg) 
                    - grad_gain * NullSpace_Jac_Lhand * joints_limit_gradient;

        cout << " Reconstr Velo_lhand_in_world is ; \n " << J_grasp_Lhand * q_dot_Lhand << endl;

    }
    else
    {
       J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

    
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        //PsdInv_Jac_Lhand    = J_grasp_Lhand.transpose() * (J_grasp_Lhand*J_grasp_Lhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Lhand);
        PsdInv_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_Jac_Lhand * J_grasp_Lhand;

        q_dot_Lhand = PsdInv_Jac_Lhand * (Velo_lhand_in_world - J_grasp_stanceleg_World * q_dot_rleg) 
                    - grad_gain * NullSpace_Jac_Lhand * joints_limit_gradient;

        cout << " Reconstr Velo_lhand_in_world is ; \n " << J_grasp_Lhand * q_dot_Lhand << endl;

    }

    qDot_Grasping_left_hand = q_dot_Lhand;

    return q_dot_Lhand;

}


// right hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::get_rh_GraspJtsVelocity_World2(  MatrixXd wld_Trsf_root,
                                                    MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Rhand_leg_World;
    MatrixXd J_grasp_Rhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd q_dot_Rhand;

    MatrixXd L_eta_rhand;
    MatrixXd W_World_rhand;

    e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);
    // e_grasp(0) = 0.1;
    // e_grasp(1) = 0.0;
    // e_grasp(2) = 0.0;
    // e_grasp(3) = 0.0;
    // e_grasp(4) = 0.0;
    // e_grasp(5) = 0.0;

    cout << " Right hand PoseError_d_H_c is ; \n " << e_grasp << endl;

    L_eta_rhand = Grasping::getInteractionMxForAxisAngle(dr_H_cr);

    W_World_rhand = Grasping::getRhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            right_Arm_Chain);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;
    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_rhand_in_world;
    MatrixXd Hand_interactionMx_world;
    Hand_interactionMx_world = L_eta_rhand; // * W_World_rhand;
    //Velo_rhand_in_world = -1.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * graspGain_rh.asDiagonal() * e_grasp;

    Velo_rhand_in_world = -3.0* graspGain_rh(0)*W_World_rhand.transpose()* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * e_grasp;

    cout << " Velo_rhand_in_world is ; \n " << Velo_rhand_in_world << endl;
    
    MatrixXd PsdInv_Jac_Rhand;
    MatrixXd NullSpace_Jac_Rhand;

    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getRightHand_JointsLimitsGradient(right_Arm_Chain, rho);

    MatrixXd J_task_rhand_World;
    

    if(StanceLeg)
    {
        J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                    right_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        PsdInv_Jac_Rhand    = J_grasp_Rhand.transpose() * (J_grasp_Rhand*J_grasp_Rhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Rhand);
        //PsdInv_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_Jac_Rhand * J_grasp_Rhand;

        q_dot_Rhand = PsdInv_Jac_Rhand * (Velo_rhand_in_world - J_grasp_stanceleg_World * q_dot_lleg) 
                    - grad_gain * NullSpace_Jac_Rhand * joints_limit_gradient;

        cout << " Reconstr Velo_rhand_in_world is ; \n " << J_grasp_Rhand * q_dot_Rhand << endl;

    }
    else
    {
       J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                   right_Arm_Chain, 
                                                                   LeftLegChain,
                                                                   RightLegChain,
                                                                   StanceLeg);

    
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        PsdInv_Jac_Rhand    = J_grasp_Rhand.transpose() * (J_grasp_Rhand*J_grasp_Rhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Rhand);
        //PsdInv_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_Jac_Rhand * J_grasp_Rhand;

        q_dot_Rhand = PsdInv_Jac_Rhand * (Velo_rhand_in_world - J_grasp_stanceleg_World * q_dot_rleg) 
                    - grad_gain * NullSpace_Jac_Rhand * joints_limit_gradient;

        cout << " Reconstr Velo_rhand_in_world is ; \n " << J_grasp_Rhand * q_dot_Rhand << endl;

    }

    qDot_Grasping_right_hand = q_dot_Rhand;

    return q_dot_Rhand;

}



VectorXd Grasping::getLeftHand_JointsLimitsGradient( iCub::iKin::iKinChain *left_Arm_Chain, double rho)
{
    double qlimit_min;
    double qlimit_max;
    VectorXd grad_JL_leftHand(left_Arm_Chain->getDOF()); 

    yarp::sig::Vector q_arm;
    q_arm = CTRL_RAD2DEG * left_Arm_Chain->getAng();

    for (unsigned int i=0; i<left_Arm_Chain->getDOF(); i++)
    {
        qlimit_min = (*left_Arm_Chain)(i).getMin() + rho * ((*left_Arm_Chain)(i).getMax() - (*left_Arm_Chain)(i).getMin());

        qlimit_max = (*left_Arm_Chain)(i).getMax() - rho * ((*left_Arm_Chain)(i).getMax() - (*left_Arm_Chain)(i).getMin());

        // computing the gradient

        if (q_arm[i] < qlimit_min)
        {
            grad_JL_leftHand(i) = (q_arm[i] - qlimit_min) * ((*left_Arm_Chain)(i).getMax() - (*left_Arm_Chain)(i).getMin());
        }
        else if (qlimit_max < q_arm[i])
        {
            grad_JL_leftHand(i) = (q_arm[i] - qlimit_max) * ((*left_Arm_Chain)(i).getMax() - (*left_Arm_Chain)(i).getMin());
        }
        else
        {
            grad_JL_leftHand(i) = 0.0;
        }
    }

    return grad_JL_leftHand;

} 

VectorXd Grasping::getRightHand_JointsLimitsGradient( iCub::iKin::iKinChain *right_Arm_Chain, double rho)
{
    double qlimit_min;
    double qlimit_max;
    VectorXd grad_JL_rightHand(right_Arm_Chain->getDOF()); 

    yarp::sig::Vector q_arm;
    q_arm = CTRL_RAD2DEG * right_Arm_Chain->getAng();

    for (unsigned int i=0; i<right_Arm_Chain->getDOF(); i++)
    {
        qlimit_min = (*right_Arm_Chain)(i).getMin() + rho * ((*right_Arm_Chain)(i).getMax() - (*right_Arm_Chain)(i).getMin());

        qlimit_max = (*right_Arm_Chain)(i).getMax() - rho * ((*right_Arm_Chain)(i).getMax() - (*right_Arm_Chain)(i).getMin());

        // computing the gradient

        if (q_arm[i] < qlimit_min)
        {
            grad_JL_rightHand(i) = (q_arm[i] - qlimit_min) * ((*right_Arm_Chain)(i).getMax() - (*right_Arm_Chain)(i).getMin());
        }
        else if (qlimit_max < q_arm[i])
        {
            grad_JL_rightHand(i) = (q_arm[i] - qlimit_max) * ((*right_Arm_Chain)(i).getMax() - (*right_Arm_Chain)(i).getMin());
        }
        else
        {
            grad_JL_rightHand(i) = 0.0;
        }
    }

    return grad_JL_rightHand;

} 


void Grasping::set_lh_GraspGain(VectorXd Gain_lh)
{
    graspGain_lh = Gain_lh;
}

void Grasping::set_rh_GraspGain(VectorXd Gain_rh)
{
    graspGain_rh = Gain_rh;
}

// ====================
// TORQUE BASED CONTROL
// ====================


// Compute joint toques
// ====================


// left hand joint torques for the grasping task  wrt. the world frame 
VectorXd Grasping::get_lh_GraspJtsTorques(  MatrixXd wld_Trsf_root,
                                            MatrixXd dl_H_cl,
                                            iCub::iKin::iKinChain *left_Arm_Chain,
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            Eigen::VectorXd q_dot_lleg,
                                            Eigen::VectorXd q_dot_rleg,
                                            bool StanceLeg) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Lhand_leg_World;
    MatrixXd J_grasp_Lhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd tau_Lhand;

    MatrixXd L_eta_lhand;
    MatrixXd W_World_lhand;

    e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);
    // e_grasp(0) = 0.0;
    // e_grasp(1) = 0.1;
    // e_grasp(2) = 0.0;
    // e_grasp(3) = 0.0;
    // e_grasp(4) = 0.0;
    // e_grasp(5) = 0.0;

    L_eta_lhand = Grasping::getInteractionMxForAxisAngle(dl_H_cl);

    W_World_lhand = Grasping::getLhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            left_Arm_Chain);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;
    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_lhand_in_world;
    MatrixXd Hand_interactionMx_world;
    // Hand_interactionMx_world = L_eta_lhand * W_World_lhand;
    //Velo_lhand_in_world = -1.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * graspGain_lh.asDiagonal() * e_grasp;

    Hand_interactionMx_world = L_eta_lhand; //.block(0,0, 3,6); // * W_World_lhand;
    //Velo_lhand_in_world = -3.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * (graspGain_lh.segment(0,3)).asDiagonal() * e_grasp.segment(0,3);
    Velo_lhand_in_world = -3.0* graspGain_lh(0)*W_World_lhand.transpose()* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * e_grasp; //.segment(0,3);
   
    MatrixXd PsdInv_Jac_Lhand;
    MatrixXd NullSpace_Jac_Lhand;

    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getLeftHand_JointsLimitsGradient(left_Arm_Chain, rho);

    MatrixXd J_task_lhand_World;
    

    if(StanceLeg)
    {
        J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        //PsdInv_Jac_Lhand    = J_grasp_Lhand.transpose() * (J_grasp_Lhand*J_grasp_Lhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Lhand);
        PsdInv_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_Jac_Lhand * J_grasp_Lhand;

        tau_Lhand = J_grasp_Lhand.transpose() * (Velo_lhand_in_world - J_grasp_stanceleg_World * q_dot_lleg) 
                    - 0.0* grad_gain * NullSpace_Jac_Lhand * joints_limit_gradient;

    }
    else
    {
       J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

    
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        //PsdInv_Jac_Lhand    = J_grasp_Lhand.transpose() * (J_grasp_Lhand*J_grasp_Lhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Lhand);
        PsdInv_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_Jac_Lhand * J_grasp_Lhand;

        tau_Lhand = J_grasp_Lhand.transpose() * (Velo_lhand_in_world - J_grasp_stanceleg_World * q_dot_rleg) 
                    - 0.0* grad_gain * NullSpace_Jac_Lhand * joints_limit_gradient;

    }

    return tau_Lhand;

}


// right hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::get_rh_GraspJtsTorques(  MatrixXd wld_Trsf_root,
                                            MatrixXd dr_H_cr,
                                            iCub::iKin::iKinChain *right_Arm_Chain,
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            Eigen::VectorXd q_dot_lleg,
                                            Eigen::VectorXd q_dot_rleg,
                                            bool StanceLeg) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Rhand_leg_World;
    MatrixXd J_grasp_Rhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd tau_Rhand;

    MatrixXd L_eta_rhand;
    MatrixXd W_World_rhand;

    e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);

    L_eta_rhand = Grasping::getInteractionMxForAxisAngle(dr_H_cr);

    W_World_rhand = Grasping::getRhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            right_Arm_Chain);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;
    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_rhand_in_world;
    MatrixXd Hand_interactionMx_world;
    Hand_interactionMx_world = L_eta_rhand; // * W_World_rhand;
    //Velo_rhand_in_world = -1.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * graspGain_rh.asDiagonal() * e_grasp;

    Velo_rhand_in_world = -3.0* graspGain_rh(0)*W_World_rhand.transpose()* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * e_grasp;
  
    MatrixXd PsdInv_Jac_Rhand;
    MatrixXd NullSpace_Jac_Rhand;

    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getRightHand_JointsLimitsGradient(right_Arm_Chain, rho);

    MatrixXd J_task_rhand_World;
    

    if(StanceLeg)
    {
        J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                    right_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        PsdInv_Jac_Rhand    = J_grasp_Rhand.transpose() * (J_grasp_Rhand*J_grasp_Rhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Rhand);
        //PsdInv_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_Jac_Rhand * J_grasp_Rhand;

        tau_Rhand = J_grasp_Rhand.transpose() * (Velo_rhand_in_world - J_grasp_stanceleg_World * q_dot_lleg) 
                    - 0.0*grad_gain * NullSpace_Jac_Rhand * joints_limit_gradient;

    }
    else
    {
       J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                   right_Arm_Chain, 
                                                                   LeftLegChain,
                                                                   RightLegChain,
                                                                   StanceLeg);

    
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        PsdInv_Jac_Rhand    = J_grasp_Rhand.transpose() * (J_grasp_Rhand*J_grasp_Rhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Rhand);
        //PsdInv_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_Jac_Rhand * J_grasp_Rhand;

        tau_Rhand = J_grasp_Rhand.transpose() * (Velo_rhand_in_world - J_grasp_stanceleg_World * q_dot_rleg) 
                    - 0.0*grad_gain * NullSpace_Jac_Rhand * joints_limit_gradient;

    }

    return tau_Rhand;

}


// global Jacobian
// left hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::get_lh_GraspJtsTorquesGlobal(   MatrixXd wld_Trsf_root,
                                                    MatrixXd dl_H_cl,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Lhand_leg_World;
    MatrixXd J_grasp_Lhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd tau_Lhand;

    MatrixXd PsdInv_grasp_Jac_Lhand;
    MatrixXd NullSpace_grasp_Jac_Lhand;

    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getLeftHand_JointsLimitsGradient(left_Arm_Chain, rho);

    e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);

    
    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        J_grap_Lhand_leg_World = Grasping::getGraspJacobianLhand_World( wld_Trsf_root,
                                                                        dl_H_cl,
                                                                        left_Arm_Chain, 
                                                                        LeftLegChain,
                                                                        RightLegChain,
                                                                        StanceLeg);

    
        J_grasp_Lhand           = J_grap_Lhand_leg_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_grap_Lhand_leg_World.block(0, n_lh, 6, n_ll);

        PsdInv_grasp_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_grasp_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_grasp_Jac_Lhand * J_grasp_Lhand;

        // q_dot_Lhand = PsdInv_grasp_Jac_Lhand * (-1.0 *graspGain_lh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_lleg) 
        //             - grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;
        tau_Lhand = J_grasp_Lhand.transpose() * (-3.0 * e_grasp - J_grasp_stanceleg_World * q_dot_lleg) 
                    - 0.0* grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;

    }
    else
    {
        J_grap_Lhand_leg_World = Grasping::getGraspJacobianLhand_World( wld_Trsf_root,
                                                                        dl_H_cl,
                                                                        left_Arm_Chain, 
                                                                        LeftLegChain,
                                                                        RightLegChain,
                                                                        StanceLeg);

    
        J_grasp_Lhand           = J_grap_Lhand_leg_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_grap_Lhand_leg_World.block(0, n_lh, 6, n_ll);

        PsdInv_grasp_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_grasp_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_grasp_Jac_Lhand * J_grasp_Lhand;

        // q_dot_Lhand = PsdInv_grasp_Jac_Lhand * (-1.0 *graspGain_lh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_rleg)
        //             - grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;
        tau_Lhand = J_grasp_Lhand.transpose() * (-3.0 * e_grasp - J_grasp_stanceleg_World * q_dot_rleg)
                    - 0.0* grad_gain * NullSpace_grasp_Jac_Lhand * joints_limit_gradient;

    }

    return tau_Lhand;

}


// right hand joint velocities for the grasping task  wrt. the world frame
VectorXd Grasping::get_rh_GraspJtsTorquesGlobal(   MatrixXd wld_Trsf_root,
                                                    MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    VectorXd e_grasp;
    MatrixXd J_grap_Rhand_leg_World;
    MatrixXd J_grasp_Rhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd tau_Rhand;

    MatrixXd PsdInv_grasp_Jac_Rhand;
    MatrixXd NullSpace_grasp_Jac_Rhand;

    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.000;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getRightHand_JointsLimitsGradient(right_Arm_Chain, rho);


    e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;

    if(StanceLeg)
    {
        J_grap_Rhand_leg_World = Grasping::getGraspJacobianRhand_World( wld_Trsf_root,
                                                                        dr_H_cr,
                                                                        right_Arm_Chain, 
                                                                        LeftLegChain,
                                                                        RightLegChain,
                                                                        StanceLeg);
    
    
        J_grasp_Rhand           = J_grap_Rhand_leg_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_grap_Rhand_leg_World.block(0, n_rh, 6, n_ll);

        PsdInv_grasp_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_grasp_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_grasp_Jac_Rhand * J_grasp_Rhand;

        // tau_Rhand = PsdInv_grasp_Jac_Rhand * (-2.0 *graspGain_rh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_lleg)
        //             - 0.0* grad_gain * NullSpace_grasp_Jac_Rhand * joints_limit_gradient;
        tau_Rhand = J_grasp_Rhand.transpose() * (-3.0 * e_grasp - J_grasp_stanceleg_World * q_dot_lleg)
                    - 0.0* grad_gain * NullSpace_grasp_Jac_Rhand * joints_limit_gradient;

        
    }
    else
    {
        J_grap_Rhand_leg_World = Grasping::getGraspJacobianRhand_World( wld_Trsf_root,
                                                                        dr_H_cr,
                                                                        right_Arm_Chain, 
                                                                        LeftLegChain,
                                                                        RightLegChain,
                                                                        StanceLeg);
    
    
        J_grasp_Rhand           = J_grap_Rhand_leg_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_grap_Rhand_leg_World.block(0, n_rh, 6, n_ll);

        PsdInv_grasp_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_grasp_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_grasp_Jac_Rhand * J_grasp_Rhand;

        // tau_Rhand = J_grasp_Rhand.transpose() * (-3.0 * graspGain_rh.asDiagonal() * e_grasp - J_grasp_stanceleg_World * q_dot_rleg)
        //             - 0.0 *grad_gain * NullSpace_grasp_Jac_Rhand * joints_limit_gradient;

        tau_Rhand = J_grasp_Rhand.transpose() * (-3.0 * e_grasp - J_grasp_stanceleg_World * q_dot_rleg)
                    - 0.0 *grad_gain * NullSpace_grasp_Jac_Rhand * joints_limit_gradient;
        
    }

    return tau_Rhand;
}

// =============================================================================================================================================
// ==============================================================================================================================================

// admittance control

// =============================================================================================================================================
// ==============================================================================================================================================

bool Grasping::InitializeLeftAdmittance(double SamplingTime, MatrixXd DynM, MatrixXd ControlMx, VectorXd Input)
{
    //
    left_hand_admittance.InitializeDynamics(SamplingTime, DynM, ControlMx, Input);

    return true;
}

bool Grasping::InitializeRightAdmittance(double SamplingTime, MatrixXd DynM, MatrixXd ControlMx, VectorXd Input)
{
    //
    right_hand_admittance.InitializeDynamics(SamplingTime, DynM, ControlMx, Input);

    return true;
}


// left hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::getLArmJointsVelocityFromAdmittance( MatrixXd wld_Trsf_root,
                                                        MatrixXd dl_H_cl,
                                                        iCub::iKin::iKinChain *left_Arm_Chain,
                                                        iCub::iKin::iKinChain *LeftLegChain,
                                                        iCub::iKin::iKinChain *RightLegChain,
                                                        Eigen::VectorXd q_dot_lleg,
                                                        Eigen::VectorXd q_dot_rleg,
                                                        bool StanceLeg,
                                                        Eigen::VectorXd lhandAppWrench) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Lhand_leg_World;
    MatrixXd J_grasp_Lhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd q_dot_Lhand;

    MatrixXd L_eta_lhand;
    MatrixXd W_World_lhand;

    e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);
    

    L_eta_lhand = Grasping::getInteractionMxForAxisAngle(dl_H_cl);

    W_World_lhand = Grasping::getLhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            left_Arm_Chain);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;
    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_lhand_in_world;
    MatrixXd Hand_interactionMx_world(6,6);
    // Hand_interactionMx_world = L_eta_lhand * W_World_lhand;
    //Velo_lhand_in_world = -1.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * graspGain_lh.asDiagonal() * e_grasp;

    // Hand_interactionMx_world = L_eta_lhand; //.block(0,0, 3,6); // * W_World_lhand;
    // //Velo_lhand_in_world = -3.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * (graspGain_lh.segment(0,3)).asDiagonal() * e_grasp.segment(0,3);
    // Velo_lhand_in_world = -3.0* graspGain_lh(0)*W_World_lhand.transpose()* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * e_grasp; //.segment(0,3);
    
    Hand_interactionMx_world.setZero();
    Hand_interactionMx_world.block<3,3>(0,0) = dl_H_cl.block<3,3>(0,0).transpose();
    Hand_interactionMx_world.block<3,3>(3,3) = MatrixXd::Identity(3,3);



    // Compute the velocity in the hand frame
    VectorXd ScaledError = lh_virtualStiffness.asDiagonal()* e_grasp;
    VectorXd lh_velocity_lh = left_hand_admittance.getRK4Solution(ScaledError - 0.5*lhandAppWrench); 

    //     
    Velo_lhand_in_world = W_World_lhand.transpose()* Hand_interactionMx_world * lh_velocity_lh;

        if (Velo_lhand_in_world.head(3).norm() > 0.02)
        {
            Velo_lhand_in_world.head(3) = Velo_lhand_in_world.head(3)/Velo_lhand_in_world.head(3).norm() * 0.02; 
        }

        if (Velo_lhand_in_world.tail(3).norm() > 0.1)
        {
            Velo_lhand_in_world.tail(3) = Velo_lhand_in_world.tail(3)/Velo_lhand_in_world.tail(3).norm() * 0.1; 
        }

    cout << " Velo_lhand_in_world is ; \n " << Velo_lhand_in_world << endl;
   
    MatrixXd PsdInv_Jac_Lhand;
    MatrixXd NullSpace_Jac_Lhand;

    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.00005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getLeftHand_JointsLimitsGradient(left_Arm_Chain, rho);

    MatrixXd J_task_lhand_World;
    

    if(StanceLeg)
    {
        J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        //PsdInv_Jac_Lhand    = J_grasp_Lhand.transpose() * (J_grasp_Lhand*J_grasp_Lhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Lhand);
        PsdInv_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_Jac_Lhand * J_grasp_Lhand;

        q_dot_Lhand = PsdInv_Jac_Lhand * (Velo_lhand_in_world - J_grasp_stanceleg_World * q_dot_lleg) 
                    - grad_gain * NullSpace_Jac_Lhand * joints_limit_gradient;

        cout << " Reconstr Velo_lhand_in_world is ; \n " << J_grasp_Lhand * q_dot_Lhand << endl;

    }
    else
    {
       J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

    
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        //PsdInv_Jac_Lhand    = J_grasp_Lhand.transpose() * (J_grasp_Lhand*J_grasp_Lhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Lhand);
        PsdInv_Jac_Lhand    = MxPsdInv.pseudoInverse(J_grasp_Lhand);
        NullSpace_Jac_Lhand = MatrixXd::Identity(n_lh, n_lh) - PsdInv_Jac_Lhand * J_grasp_Lhand;

        q_dot_Lhand = PsdInv_Jac_Lhand * (Velo_lhand_in_world - J_grasp_stanceleg_World * q_dot_rleg) 
                    - grad_gain * NullSpace_Jac_Lhand * joints_limit_gradient;

        cout << " Reconstr Velo_lhand_in_world is ; \n " << J_grasp_Lhand * q_dot_Lhand << endl;

    }

        for (int i = 0; i<left_Arm_Chain->getDOF(); i++)
        { 
            if (fabs(q_dot_Lhand(i)) > 0.15)
            {
                q_dot_Lhand(i) = q_dot_Lhand(i)/fabs(q_dot_Lhand(i)) * 0.15; 
            }
        }

    qDot_Grasping_left_hand = q_dot_Lhand;

    return q_dot_Lhand;

}


// right hand joint velocities for the grasping task  wrt. the world frame 
VectorXd Grasping::getRArmJointsVelocityFromAdmittance( MatrixXd wld_Trsf_root,
                                                        MatrixXd dr_H_cr,
                                                        iCub::iKin::iKinChain *right_Arm_Chain,
                                                        iCub::iKin::iKinChain *LeftLegChain,
                                                        iCub::iKin::iKinChain *RightLegChain,
                                                        Eigen::VectorXd q_dot_lleg,
                                                        Eigen::VectorXd q_dot_rleg,
                                                        bool StanceLeg,
                                                        Eigen::VectorXd rhandAppWrench) 
{
    
    VectorXd e_grasp;
    MatrixXd J_grap_Rhand_leg_World;
    MatrixXd J_grasp_Rhand;
    MatrixXd J_grasp_stanceleg_World;
    VectorXd q_dot_Rhand;

    MatrixXd L_eta_rhand;
    MatrixXd W_World_rhand;

    e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);

    L_eta_rhand = Grasping::getInteractionMxForAxisAngle(dr_H_cr);

    W_World_rhand = Grasping::getRhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            right_Arm_Chain);

    // compute the pseudo inverse matrix
    MatrixPseudoInverse MxPsdInv;
    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_rhand_in_world;
    MatrixXd Hand_interactionMx_world(6,6);
    // Hand_interactionMx_world = L_eta_rhand; // * W_World_rhand;
    // //Velo_rhand_in_world = -1.0* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * graspGain_rh.asDiagonal() * e_grasp;
    // Velo_rhand_in_world = -3.0* graspGain_rh(0)*W_World_rhand.transpose()* MxPsdInv.pseudoInverse(Hand_interactionMx_world) * e_grasp;

    Hand_interactionMx_world.setZero();
    Hand_interactionMx_world.block<3,3>(0,0) = dr_H_cr.block<3,3>(0,0).transpose();
    Hand_interactionMx_world.block<3,3>(3,3) = MatrixXd::Identity(3,3);

    // Compute the velocity in the hand frame
    VectorXd ScaledError = rh_virtualStiffness.asDiagonal()* e_grasp;
    //VectorXd ScaledError = e_grasp;
    cout << " ScaledError ok ; \n "<< endl;

    VectorXd rh_velocity_rh =  right_hand_admittance.getRK4Solution(ScaledError - 0.0*rhandAppWrench); 
    cout << " rh_velocity_rh ok ; \n "<< endl;

    Velo_rhand_in_world = W_World_rhand.transpose()* Hand_interactionMx_world * rh_velocity_rh;

    if (Velo_rhand_in_world.head(3).norm() > 0.02)
    {
        Velo_rhand_in_world.head(3) = Velo_rhand_in_world.head(3)/Velo_rhand_in_world.head(3).norm() * 0.02; 
    }
    
    if (Velo_rhand_in_world.tail(3).norm() > 0.1)
    {
        Velo_rhand_in_world.tail(3) = Velo_rhand_in_world.tail(3)/Velo_rhand_in_world.tail(3).norm() * 0.1; 
    }

     cout << " Velo_rhand_in_world ok ; \n "<< endl;

   
    
    MatrixXd PsdInv_Jac_Rhand;
    MatrixXd NullSpace_Jac_Rhand;

    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();
    double grad_gain = 0.00005;
    double rho = 0.2;

    VectorXd joints_limit_gradient;

    joints_limit_gradient = Grasping::getRightHand_JointsLimitsGradient(right_Arm_Chain, rho);

    MatrixXd J_task_rhand_World;
    

    if(StanceLeg)
    {
        J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                    right_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        PsdInv_Jac_Rhand    = J_grasp_Rhand.transpose() * (J_grasp_Rhand*J_grasp_Rhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Rhand);
        //PsdInv_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_Jac_Rhand * J_grasp_Rhand;

        q_dot_Rhand = PsdInv_Jac_Rhand * (Velo_rhand_in_world - J_grasp_stanceleg_World * q_dot_lleg) 
                    - grad_gain * NullSpace_Jac_Rhand * joints_limit_gradient;

        cout << " Reconstr Velo_rhand_in_world is ; \n " << J_grasp_Rhand * q_dot_Rhand << endl;

    }
    else
    {
       J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                   right_Arm_Chain, 
                                                                   LeftLegChain,
                                                                   RightLegChain,
                                                                   StanceLeg);

    
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        PsdInv_Jac_Rhand    = J_grasp_Rhand.transpose() * (J_grasp_Rhand*J_grasp_Rhand.transpose()).inverse(); // MxPsdInv.pseudoInverse(J_grasp_Rhand);
        //PsdInv_Jac_Rhand    = MxPsdInv.pseudoInverse(J_grasp_Rhand);
        NullSpace_Jac_Rhand = MatrixXd::Identity(n_rh, n_rh) - PsdInv_Jac_Rhand * J_grasp_Rhand;

        q_dot_Rhand = PsdInv_Jac_Rhand * (Velo_rhand_in_world - J_grasp_stanceleg_World * q_dot_rleg) 
                    - grad_gain * NullSpace_Jac_Rhand * joints_limit_gradient;

        cout << " Reconstr Velo_rhand_in_world is ; \n " << J_grasp_Rhand * q_dot_Rhand << endl;

    }

        for (int i = 0; i<right_Arm_Chain->getDOF(); i++)
        { 
            if (fabs(q_dot_Rhand(i)) > 0.15)
            {
                q_dot_Rhand(i) = q_dot_Rhand(i)/fabs(q_dot_Rhand(i)) * 0.15; 
            }
        }

    qDot_Grasping_right_hand = q_dot_Rhand;

    return q_dot_Rhand;

}


// void Grasping::setVirtualInertia(VectorXd l_inertia, VectorXd r_inertia)
// {
//     lh_virtualInertia = l_inertia.asDiagonal();
//     rh_virtualInertia = r_inertia.asDiagonal();
// }

// void Grasping::setVirtualDamping(VectorXd l_damping, VectorXd r_damping)
// {
//     lh_virtualDamping = l_damping.asDiagonal();
//     rh_virtualDamping = r_damping.asDiagonal();
// }

void Grasping::setVirtualStiffness(VectorXd l_stiffness, VectorXd r_stiffness)
{
    lh_virtualStiffness = l_stiffness;
    rh_virtualStiffness = r_stiffness;
}


bool Grasping::UpdateTaskTransformations( MatrixXd world_H_root,
                                          yarp::sig::Vector Des_lhandPose_in_ref,   // reference frame could be the root or the world frame
                                          yarp::sig::Vector Des_rhandPose_in_ref,
                                          RobotKinematics &botKin,
                                          bool useWorldRef)
{
    //
    if(!useWorldRef)
    {
        // Homogeneous tranformation of desired hands frame wrt. the root frame
        Matrix4d HTrsf_DesLh_2_root = Pose2Matrix.yarpPose2eigenHmatrix(Des_lhandPose_in_ref);
        Matrix4d HTrsf_DesRh_2_root = Pose2Matrix.yarpPose2eigenHmatrix(Des_rhandPose_in_ref);

        // Homogeneous tranformation of current hands frmaes wrt. the root frame
        Matrix4d HTrsf_CurLh_2_root = Pose2Matrix.yarpPose2eigenHmatrix(botKin.Left_Arm_Chain->EndEffPose());
        Matrix4d HTrsf_CurRh_2_root = Pose2Matrix.yarpPose2eigenHmatrix(botKin.Right_Arm_Chain->EndEffPose());

        // Homogeneous transformation of current hands frames wrt. the desired hands frames
        this->HTrsf_CurLh_2_DesLh = HTrsf_DesLh_2_root.inverse() * HTrsf_CurLh_2_root;
        this->HTrsf_CurRh_2_DesRh = HTrsf_DesRh_2_root.inverse() * HTrsf_CurRh_2_root;
        //
    }
    else
    {
        // Homogeneous tranformation of desired hands frame wrt. the root frame
        Matrix4d HTrsf_DesLh_2_world = Pose2Matrix.yarpPose2eigenHmatrix(Des_lhandPose_in_ref);
        Matrix4d HTrsf_DesRh_2_world = Pose2Matrix.yarpPose2eigenHmatrix(Des_rhandPose_in_ref);

        // Homogeneous tranformation of current hands frmaes wrt. the root frame
        Matrix4d HTrsf_CurLh_2_root  = Pose2Matrix.yarpPose2eigenHmatrix(botKin.Left_Arm_Chain->EndEffPose());
        Matrix4d HTrsf_CurRh_2_root  = Pose2Matrix.yarpPose2eigenHmatrix(botKin.Right_Arm_Chain->EndEffPose());

        //
        Matrix4d HTrsf_CurLh_2_world = world_H_root * HTrsf_CurLh_2_root;
        Matrix4d HTrsf_CurRh_2_world = world_H_root * HTrsf_CurRh_2_root;

        // Homogeneous transformation of current hands frames wrt. the desired hands frames
        this->HTrsf_CurLh_2_DesLh = HTrsf_DesLh_2_world.inverse() * HTrsf_CurLh_2_world;
        this->HTrsf_CurRh_2_DesRh = HTrsf_DesRh_2_world.inverse() * HTrsf_CurRh_2_world;
        //
    }

    return true;
}               
    
// ==================================
// left hand joint Hessian and gradient for the grasping task  wrt. the world frame 
bool Grasping::get_lh_GraspWorld2_HessianGradient(  MatrixXd wld_Trsf_root,
                                                    MatrixXd dl_H_cl,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    //
    int n_lh = left_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();

    MatrixXd J_grasp_Lhand(6, n_lh);
    MatrixXd J_grasp_stanceleg_World(6, n_ll);

    VectorXd e_grasp = Grasping::getPoseError_d_H_c(dl_H_cl);

    MatrixXd L_eta_lhand = Grasping::getInteractionMxForAxisAngle(dl_H_cl);

    MatrixXd W_World_lhand = Grasping::getLhand_World_VeloTwistMx(   wld_Trsf_root,
                                                                    left_Arm_Chain);

    // compute the velocity of the hand with respect to the world frame
    VectorXd Velo_lhand_in_world;
    MatrixXd Hand_interactionMx_world(6,6);
    Hand_interactionMx_world.setZero();
    Hand_interactionMx_world.block<3,3>(0,0) = dl_H_cl.block<3,3>(0,0).transpose();
    Hand_interactionMx_world.block<3,3>(3,3) = MatrixXd::Identity(3,3);
    Velo_lhand_in_world = -3.0* graspGain_lh(0)*W_World_lhand.transpose()* Hand_interactionMx_world * e_grasp;

    
    //
    MatrixXd J_task_lhand_World(6, n_lh+n_ll);
    

    if(StanceLeg)
    {
        J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        // Hessian Matrix and Gradient vector
        // Eigen::MatrixXd LWJ_lhand = L_eta_lhand * W_World_lhand * J_grasp_Lhand;
        // Eigen::VectorXd temp_LWJl_qDotl_ke = L_eta_lhand * W_World_lhand * J_grasp_stanceleg_World * q_dot_lleg -3.0* graspGain_lh.asDiagonal() * e_grasp;
        //
        Eigen::MatrixXd LWJ_lhand = J_grasp_Lhand;
        Eigen::VectorXd temp_LWJl_qDotl_ke = J_grasp_stanceleg_World * q_dot_lleg - Velo_lhand_in_world;
        //
        //
        Gsp_Hessian_matrix_lhand = LWJ_lhand.transpose() * LWJ_lhand + Weight_qDot.asDiagonal() * Eigen::MatrixXd::Identity(n_lh,n_lh);
        //
        Gsp_Gradient_vector_lhand = LWJ_lhand.transpose() * temp_LWJl_qDotl_ke;

        
    }
    else
    {
       J_task_lhand_World = Grasping::getLhand_world_TaskJacobian( wld_Trsf_root,
                                                                    left_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

    
        J_grasp_Lhand           = J_task_lhand_World.block(0, 0, 6, n_lh);
        J_grasp_stanceleg_World = J_task_lhand_World.block(0, n_lh, 6, n_ll);

        // Hessian Matrix and Gradient vector
        // Eigen::MatrixXd LWJ_lhand = L_eta_lhand * W_World_lhand * J_grasp_Lhand;
        // Eigen::VectorXd temp_LWJl_qDotl_ke = L_eta_lhand * W_World_lhand * J_grasp_stanceleg_World * q_dot_rleg + 3.0* graspGain_lh.asDiagonal() * e_grasp;
        //
        //
        Eigen::MatrixXd LWJ_lhand = J_grasp_Lhand;
        Eigen::VectorXd temp_LWJl_qDotl_ke = J_grasp_stanceleg_World * q_dot_rleg - Velo_lhand_in_world;
        //
        Gsp_Hessian_matrix_lhand = LWJ_lhand.transpose() * LWJ_lhand + Weight_qDot.asDiagonal() * Eigen::MatrixXd::Identity(n_lh,n_lh);
        //
        Gsp_Gradient_vector_lhand = LWJ_lhand.transpose() * temp_LWJl_qDotl_ke;

    }

    return true;

}


// right hand joint velocities for the grasping task  wrt. the world frame 
bool Grasping::get_rh_GraspWorld2_HessianGradient(  MatrixXd wld_Trsf_root,
                                                    MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg) 
{
    
    //    
    int n_rh = right_Arm_Chain->getDOF();
    int n_ll = LeftLegChain->getDOF();

    MatrixXd J_grasp_Rhand(6, n_rh);
    MatrixXd J_grasp_stanceleg_World(6, n_ll);

    VectorXd e_grasp = Grasping::getPoseError_d_H_c(dr_H_cr);

    MatrixXd L_eta_rhand = Grasping::getInteractionMxForAxisAngle(dr_H_cr);

    MatrixXd W_World_rhand = Grasping::getRhand_World_VeloTwistMx(   wld_Trsf_root,
                                                            right_Arm_Chain);

    

    MatrixXd J_task_rhand_World(6, n_rh+n_ll);
    

    if(StanceLeg)
    {
        J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                    right_Arm_Chain, 
                                                                    LeftLegChain,
                                                                    RightLegChain,
                                                                    StanceLeg);

   
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        // Hessian Matrix and Gradient vector
        Eigen::MatrixXd LWJ_rhand = L_eta_rhand * W_World_rhand * J_grasp_Rhand;
        Eigen::VectorXd temp_LWJl_qDotl_ke = L_eta_rhand * W_World_rhand * J_grasp_stanceleg_World * q_dot_lleg +3.0* graspGain_rh.asDiagonal() * e_grasp;
        //
        Gsp_Hessian_matrix_rhand = LWJ_rhand.transpose() * LWJ_rhand + Weight_qDot.asDiagonal() * Eigen::MatrixXd::Identity(n_rh,n_rh);
        //
        Gsp_Gradient_vector_rhand = LWJ_rhand.transpose() * temp_LWJl_qDotl_ke;


    }
    else
    {
       J_task_rhand_World = Grasping::getRhand_world_TaskJacobian( wld_Trsf_root,
                                                                   right_Arm_Chain, 
                                                                   LeftLegChain,
                                                                   RightLegChain,
                                                                   StanceLeg);

    
        J_grasp_Rhand           = J_task_rhand_World.block(0, 0, 6, n_rh);
        J_grasp_stanceleg_World = J_task_rhand_World.block(0, n_rh, 6, n_ll);

        // Hessian Matrix and Gradient vector
        Eigen::MatrixXd LWJ_rhand          = L_eta_rhand * W_World_rhand * J_grasp_Rhand;
        Eigen::VectorXd temp_LWJl_qDotl_ke = L_eta_rhand * W_World_rhand * J_grasp_stanceleg_World * q_dot_rleg +3.0* graspGain_rh.asDiagonal() * e_grasp;
        //
        Gsp_Hessian_matrix_rhand  = LWJ_rhand.transpose() * LWJ_rhand + Weight_qDot.asDiagonal() * Eigen::MatrixXd::Identity(n_rh,n_rh);
        //
        Gsp_Gradient_vector_rhand = LWJ_rhand.transpose() * temp_LWJl_qDotl_ke;

    }

    return true;

}

// Inequality constraints
//
bool Grasping::get_lhand_InequalityConstraints( double SamplingTime,
                                                iCub::iKin::iKinChain *left_Arm_Chain,
                                                Eigen::VectorXd velocitySaturationLimit)
{
    //
    int n_lh = left_Arm_Chain->getDOF();

    // Velocity limit matrix and Vector
    Eigen::MatrixXd VelocityLimitsMx(2*n_lh, n_lh);
    VelocityLimitsMx.topRows(n_lh)    =  Eigen::MatrixXd::Identity(n_lh, n_lh);
    VelocityLimitsMx.bottomRows(n_lh) = -Eigen::MatrixXd::Identity(n_lh, n_lh);
    //
    Eigen::VectorXd VelocityLimitsVec(2*n_lh);
    VelocityLimitsVec.head(n_lh) = velocitySaturationLimit;
    VelocityLimitsVec.tail(n_lh) = velocitySaturationLimit;

    // Joint position matrix and vector
    // matrix
    Eigen::MatrixXd JointsLimitsMx(2*n_lh, n_lh);
    JointsLimitsMx.topRows(n_lh)    =  SamplingTime * Eigen::MatrixXd::Identity(n_lh, n_lh);
    JointsLimitsMx.bottomRows(n_lh) = -SamplingTime * Eigen::MatrixXd::Identity(n_lh, n_lh);
    //
    // vector
    Eigen::VectorXd maxJointLimits_lhand(left_Arm_Chain->getDOF());
    Eigen::VectorXd minJointLimits_lhand(left_Arm_Chain->getDOF());

    Eigen::VectorXd larm_joints(left_Arm_Chain->getDOF());

    // extract the joint limits
    for (unsigned int i=0; i<left_Arm_Chain->getDOF(); i++)
    {    
        minJointLimits_lhand(i)=(*left_Arm_Chain)(i).getMin();
        maxJointLimits_lhand(i)=(*left_Arm_Chain)(i).getMax();

        larm_joints(i)=(*left_Arm_Chain)[i].getAng();

    }
    //
    Eigen::VectorXd JointsLimitsVec(2*n_lh);
    JointsLimitsVec.head(n_lh) =  maxJointLimits_lhand - larm_joints;
    JointsLimitsVec.tail(n_lh) = -minJointLimits_lhand + larm_joints;


    // Matrix
    Inequality_matrix_lhand.block(     0, 0, 2*n_lh,n_lh) = VelocityLimitsMx;
    Inequality_matrix_lhand.block(2*n_lh, 0, 2*n_lh,n_lh) = JointsLimitsMx;

    // Vector
    Inequality_vector_lhand.segment(     0, 2*n_lh) = VelocityLimitsVec;
    Inequality_vector_lhand.segment(2*n_lh, 2*n_lh) = JointsLimitsVec;

    //
    return true;

}

// Inequality constraints
//
bool Grasping::get_rhand_InequalityConstraints( double SamplingTime,
                                                iCub::iKin::iKinChain *right_Arm_Chain,
                                                Eigen::VectorXd velocitySaturationLimit)
{
    //
    int n_rh = right_Arm_Chain->getDOF();

    // Velocity limit matrix and Vector
    Eigen::MatrixXd VelocityLimitsMx(2*n_rh, n_rh);
    VelocityLimitsMx.topRows(n_rh)    =  Eigen::MatrixXd::Identity(n_rh, n_rh);
    VelocityLimitsMx.bottomRows(n_rh) = -Eigen::MatrixXd::Identity(n_rh, n_rh);
    //
    Eigen::VectorXd VelocityLimitsVec(2*n_rh);
    VelocityLimitsVec.head(n_rh) = velocitySaturationLimit;
    VelocityLimitsVec.tail(n_rh) = velocitySaturationLimit;

    // Joint position matrix and vector
    // matrix
    Eigen::MatrixXd JointsLimitsMx(2*n_rh, n_rh);
    JointsLimitsMx.topRows(n_rh)    =  SamplingTime * Eigen::MatrixXd::Identity(n_rh, n_rh);
    JointsLimitsMx.bottomRows(n_rh) = -SamplingTime * Eigen::MatrixXd::Identity(n_rh, n_rh);
    //
    // vector
    Eigen::VectorXd maxJointLimits_rhand(right_Arm_Chain->getDOF());
    Eigen::VectorXd minJointLimits_rhand(right_Arm_Chain->getDOF());

    Eigen::VectorXd rarm_joints(right_Arm_Chain->getDOF());

    // extract the joint limits
    for (unsigned int i=0; i<right_Arm_Chain->getDOF(); i++)
    {    
        minJointLimits_rhand(i)=(*right_Arm_Chain)(i).getMin();
        maxJointLimits_rhand(i)=(*right_Arm_Chain)(i).getMax();

        rarm_joints(i)=(*right_Arm_Chain)[i].getAng();

    }

    //
    Eigen::VectorXd JointsLimitsVec(2*n_rh);
    JointsLimitsVec.head(n_rh) =  maxJointLimits_rhand - rarm_joints;
    JointsLimitsVec.tail(n_rh) = -minJointLimits_rhand + rarm_joints;

    // Matrix
    Inequality_matrix_rhand.block(     0, 0, 2*n_rh,n_rh) = VelocityLimitsMx;
    Inequality_matrix_rhand.block(2*n_rh, 0, 2*n_rh,n_rh) = JointsLimitsMx;

    // Vector
    Inequality_vector_rhand.segment(     0, 2*n_rh) = VelocityLimitsVec;
    Inequality_vector_rhand.segment(2*n_rh, 2*n_rh) = JointsLimitsVec;

    //
    return true;

}
 

// ====================================== INITIALIZATION ========================================================

// left hand joint velocities for the grasping task  wrt. the world frame 
bool Grasping::InitializeQPsolver_lh_GraspWorld2(  MatrixXd wld_Trsf_root,
                                                    MatrixXd dl_H_cl,
                                                    iCub::iKin::iKinChain *left_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg,
                                                    double SamplingTime,
                                                    Eigen::VectorXd velocitySaturationLimit) 
{
    //
    bool ok = false;

    if(!(ok = this->get_lh_GraspWorld2_HessianGradient( wld_Trsf_root,
                                                        dl_H_cl,
                                                        left_Arm_Chain,
                                                        LeftLegChain,
                                                        RightLegChain,
                                                        q_dot_lleg,
                                                        q_dot_rleg,
                                                        StanceLeg)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        return false;
    }
    //
    if(!(ok = ok && this->get_lhand_InequalityConstraints( SamplingTime,
                                                            left_Arm_Chain,
                                                            velocitySaturationLimit)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        return false;
    }

    //USING_NAMESPACE_QPOASES
    //int_t nWSRAng = 200;
    Options QP_qfc_OptionToSet;
    QP_qfc_OptionToSet.setToDefault();
    //QP_qfc_OptionToSet.setToMPC();
    QP_qfc_OptionToSet.printLevel = PL_LOW; //PL_LOW;
    QP_qfc_OptionToSet.enableFlippingBounds = BT_TRUE;
    QP_qfc_OptionToSet.enableRegularisation = BT_TRUE;
    QP_qfc_OptionToSet.enableEqualities     = BT_TRUE;

    // Instantiation of a QP Solver object for the high level priority
    qpOSolver_lhand =  new QPOasesSolver;
    qpOSolver_lhand->InitialiseQPSol(Gsp_Hessian_matrix_lhand, Gsp_Gradient_vector_lhand, Inequality_matrix_lhand, Inequality_vector_lhand);
    qpOSolver_lhand->setSolverOptions(QP_qfc_OptionToSet);

    // Vector qDot_lhand
    Eigen::VectorXd qDot_lhand = qpOSolver_lhand->qpOasesSolver(  Gsp_Hessian_matrix_lhand,               // 
                                                                    Gsp_Gradient_vector_lhand, 
                                                                    Inequality_matrix_lhand, 
                                                                    Inequality_vector_lhand);


    return true;
}



// right hand joint velocities for the grasping task  wrt. the world frame 
bool Grasping::InitializeQPsolver_rh_GraspWorld2(  MatrixXd wld_Trsf_root,
                                                    MatrixXd dr_H_cr,
                                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                                    iCub::iKin::iKinChain *LeftLegChain,
                                                    iCub::iKin::iKinChain *RightLegChain,
                                                    Eigen::VectorXd q_dot_lleg,
                                                    Eigen::VectorXd q_dot_rleg,
                                                    bool StanceLeg,
                                                    double SamplingTime,
                                                    Eigen::VectorXd velocitySaturationLimit)
{
    //
    bool ok = false;

    if(!(ok = this->get_rh_GraspWorld2_HessianGradient( wld_Trsf_root,
                                                        dr_H_cr,
                                                        right_Arm_Chain,
                                                        LeftLegChain,
                                                        RightLegChain,
                                                        q_dot_lleg,
                                                        q_dot_rleg,
                                                        StanceLeg)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        return false;
    }
    //
     if(!(ok = ok && this->get_rhand_InequalityConstraints( SamplingTime,
                                                            right_Arm_Chain,
                                                            velocitySaturationLimit)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        return false;
    }
    //
    //USING_NAMESPACE_QPOASES
    //int_t nWSRAng = 200;
    Options QP_qfc_OptionToSet;
    QP_qfc_OptionToSet.setToDefault();
    //QP_qfc_OptionToSet.setToMPC();
    QP_qfc_OptionToSet.printLevel = PL_LOW; //PL_LOW;
    QP_qfc_OptionToSet.enableFlippingBounds = BT_TRUE;
    QP_qfc_OptionToSet.enableRegularisation = BT_TRUE;
    QP_qfc_OptionToSet.enableEqualities     = BT_TRUE;

    // Instantiation of a QP Solver object for the high level priority
    qpOSolver_rhand =  new QPOasesSolver;
    qpOSolver_rhand->InitialiseQPSol(Gsp_Hessian_matrix_rhand, Gsp_Gradient_vector_rhand, Inequality_matrix_rhand, Inequality_vector_rhand);
    qpOSolver_rhand->setSolverOptions(QP_qfc_OptionToSet);

    // Vector qDot_lhand
    Eigen::VectorXd qDot_rhand = qpOSolver_rhand->qpOasesSolver(    Gsp_Hessian_matrix_rhand,               // 
                                                                    Gsp_Gradient_vector_rhand, 
                                                                    Inequality_matrix_rhand, 
                                                                    Inequality_vector_rhand);


    return true;
}


// ====================================== GET QP SOLUTION ========================================================

// left hand joint velocities for the grasping task  wrt. the world frame 
Eigen::VectorXd Grasping::GetQPsolution_lh_GraspWorld2(  MatrixXd wld_Trsf_root,
                                                        MatrixXd dl_H_cl,
                                                        iCub::iKin::iKinChain *left_Arm_Chain,
                                                        iCub::iKin::iKinChain *LeftLegChain,
                                                        iCub::iKin::iKinChain *RightLegChain,
                                                        Eigen::VectorXd q_dot_lleg,
                                                        Eigen::VectorXd q_dot_rleg,
                                                        bool StanceLeg,
                                                        double SamplingTime,
                                                        Eigen::VectorXd velocitySaturationLimit) 
{
    //
    bool ok = false;

    if(!(ok = this->get_lh_GraspWorld2_HessianGradient( wld_Trsf_root,
                                                        dl_H_cl,
                                                        left_Arm_Chain,
                                                        LeftLegChain,
                                                        RightLegChain,
                                                        q_dot_lleg,
                                                        q_dot_rleg,
                                                        StanceLeg)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        // return -1;
    }
    //
    if(!(ok = ok && this->get_lhand_InequalityConstraints( SamplingTime,
                                                            left_Arm_Chain,
                                                            velocitySaturationLimit)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        // return -1;
    }

    //USING_NAMESPACE_QPOASES
    //int_t nWSRAng = 200;
    int nWSR = 400;
    qpOSolver_lhand->setnbWorkingSetRecalculation(nWSR);

    // Vector qDot_lhand
    Eigen::VectorXd qDot_lhand = qpOSolver_lhand->qpOasesSolver(  Gsp_Hessian_matrix_lhand,               // 
                                                                    Gsp_Gradient_vector_lhand, 
                                                                    Inequality_matrix_lhand, 
                                                                    Inequality_vector_lhand);

    //
    qDot_Grasping_left_hand = qDot_lhand;
    
    return qDot_lhand;
}



// right hand joint velocities for the grasping task  wrt. the world frame 
Eigen::VectorXd Grasping::GetQPsolution_rh_GraspWorld2(  MatrixXd wld_Trsf_root,
                                                        MatrixXd dr_H_cr,
                                                        iCub::iKin::iKinChain *right_Arm_Chain,
                                                        iCub::iKin::iKinChain *LeftLegChain,
                                                        iCub::iKin::iKinChain *RightLegChain,
                                                        Eigen::VectorXd q_dot_lleg,
                                                        Eigen::VectorXd q_dot_rleg,
                                                        bool StanceLeg,
                                                        double SamplingTime,
                                                        Eigen::VectorXd velocitySaturationLimit)
{
    //
    bool ok = false;

    if(!(ok = this->get_rh_GraspWorld2_HessianGradient( wld_Trsf_root,
                                                        dr_H_cr,
                                                        right_Arm_Chain,
                                                        LeftLegChain,
                                                        RightLegChain,
                                                        q_dot_lleg,
                                                        q_dot_rleg,
                                                        StanceLeg)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        // return -1;
    }
    //
     if(!(ok = ok && this->get_rhand_InequalityConstraints( SamplingTime,
                                                            right_Arm_Chain,
                                                            velocitySaturationLimit)))
    {
        printf("Failed to compute the Hessian and gradient vector \n");

        // return -1;
    }
    //
    //USING_NAMESPACE_QPOASES
    int nWSR = 400;
    qpOSolver_rhand->setnbWorkingSetRecalculation(nWSR);

    // Vector qDot_lhand
    Eigen::VectorXd qDot_rhand = qpOSolver_rhand->qpOasesSolver(    Gsp_Hessian_matrix_rhand,               // 
                                                                    Gsp_Gradient_vector_rhand, 
                                                                    Inequality_matrix_rhand, 
                                                                    Inequality_vector_rhand);
    //
    qDot_Grasping_right_hand = qDot_rhand;

    return qDot_rhand;
}