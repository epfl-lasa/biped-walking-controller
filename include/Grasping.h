/**
 * upper limbs Kinematics
 */ 

#ifndef Grasping_H
#define Grasping_H

#include <cmath>
#include <iostream>
#include <iomanip>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>


#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "CpMath_Utilities.h"
#include "RobotModel.h"

#include "QPOasesSolver.hpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class Grasping
{


	MatrixXd Rot_icubFoot_GenFoot;
	
	public :

	VectorXd d_eta_c;
	MatrixXd InteractionMx_Mu_Theta;

    MatrixXd Jacobian_Lhand_root;
    MatrixXd Jacobian_Rhand_root;

    MatrixXd Jacobian_Lleg_root;
    MatrixXd Jacobian_Rleg_root;

    
    MatrixXd RootVeloTwistMx_lhand_lleg;
    MatrixXd RootVeloTwistMx_rhand_lleg;
    MatrixXd RootVeloTwistMx_lhand_rleg;
    MatrixXd RootVeloTwistMx_rhand_rleg;
    MatrixXd Rot6x6_root_Glstance;
    MatrixXd Rot6x6_root_Grstance;
    MatrixXd Root_LeftStanceLeg_Jacobian;   
    MatrixXd Root_RightStanceLeg_Jacobian;

    MatrixXd Lhand_virtBase_TaskJacobian;
    MatrixXd Rhand_virtBase_TaskJacobian;
    MatrixXd Lhand_StanceLeg_TaskJacobian;
    MatrixXd Rhand_StanceLeg_TaskJacobian;

    MatrixXd Glstance_lhand_VeloTwistMx;
    MatrixXd Glstance_rhand_VeloTwistMx;
    MatrixXd Grstance_lhand_VeloTwistMx;
    MatrixXd Grstance_rhand_VeloTwistMx;

    MatrixXd Lhand_GenStanceLegVeloTwistMx;
    MatrixXd Rhand_GenStanceLegVeloTwistMx;

    MatrixXd GraspJacobianLhand_GenStanceLeg;
    MatrixXd GraspJacobianRhand_GenStanceLeg;

    // wrt. the world frame
    MatrixXd World_lhand_VeloTwistMx;       
    MatrixXd World_rhand_VeloTwistMx;
    MatrixXd Lhand_World_VeloTwistMx;   
    MatrixXd Rhand_World_VeloTwistMx;
    MatrixXd Lhand_VBase_TskJac_World;
    MatrixXd Rhand_VBase_TskJac_World;

    MatrixXd Rot6x6_root_World;
    MatrixXd RootVeloTwistMx_lh_World;   
    MatrixXd RootVeloTwistMx_rh_World; 

    MatrixXd GraspJacobianLhand_World;
    MatrixXd GraspJacobianRhand_World;

    VectorXd graspGain_lh;
    VectorXd graspGain_rh;

    VectorXd qDot_Grasping_left_hand;
    VectorXd qDot_Grasping_right_hand;
    VectorXd qDot_Grasping_bimanual;

    //
    yarp::sig::Vector desired_lhand_pose_in_root;
    yarp::sig::Vector desired_rhand_pose_in_root;
    yarp::sig::Vector Desired_Left_Arm_PoseAsAxisAngles;
    yarp::sig::Vector Desired_Right_Arm_PoseAsAxisAngles;

    //
    // Homogenaous transfromation of the current hand frame wrt to the desired one
    Matrix4d HTrsf_CurLh_2_DesLh;
    Matrix4d HTrsf_CurRh_2_DesRh;

    KinConverter Pose2Matrix;

    Grasping();

    ~Grasping();

    void Inititialize(iCub::iKin::iKinChain *left_Arm_Chain,
                                            iCub::iKin::iKinChain *right_Arm_Chain);

    // Compute the Vector of feature error
    VectorXd getPoseError_d_H_c(MatrixXd d_H_c);

    // Compute the Vector of feature error
    VectorXd getPoseError_d_H_c2(MatrixXd d_H_c);


    // 
    MatrixXd getInteractionMxForAxisAngle(MatrixXd d_H_c);  

    // 
    MatrixXd getInteractionMxForAxisAngle2(MatrixXd d_H_c);  

    // root jocabian matrix associated with left stance leg
	  MatrixXd getRoot_LeftStanceLeg_Jacobian(iCub::iKin::iKinChain *LeftLegChain);

	  // root jocabian matrix associated with right stance leg
	  MatrixXd getRoot_RightStanceLeg_Jacobian(iCub::iKin::iKinChain *RightLegChain);

    // Velocity twist matrix of a General stance foot frame (taken as the wolrd frame) wrt. the left hand frame  
	  MatrixXd getLhand_GenStanceLegVeloTwistMx(	iCub::iKin::iKinChain *left_Arm_Chain, 
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                bool StanceLeg);

	// Velocity twist matrix of a General stance foot frame (taken as the wolrd frame) wrt. the right hand frame
	MatrixXd getRhand_GenStanceLegVeloTwistMx(	iCub::iKin::iKinChain *right_Arm_Chain, 
                                              iCub::iKin::iKinChain *LeftLegChain,
                                              iCub::iKin::iKinChain *RightLegChain,
                                              bool StanceLeg);

	// Task Jacobian of the left hand associated to the robot root link with left stance leg
	MatrixXd getLhand_virtBase_TaskJacobian(  iCub::iKin::iKinChain *left_Arm_Chain, 
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            bool StanceLeg);

	// Task Jacobian of the right hand associated to the robot root link with left stance leg
	MatrixXd getRhand_virtBase_TaskJacobian(iCub::iKin::iKinChain *right_Arm_Chain, 
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            bool StanceLeg);

	// Task jacobian of left hand wrt. the stance legs (with General stance foot taken as world frame)
	MatrixXd getLhand_StanceLeg_TaskJacobian( iCub::iKin::iKinChain *left_Arm_Chain, 
                                              iCub::iKin::iKinChain *LeftLegChain,
                                              iCub::iKin::iKinChain *RightLegChain,
                                              bool StanceLeg);

	// Task jacobian of right hand wrt. the stance legs (with General stance foot taken as world frame)
	MatrixXd getRhand_StanceLeg_TaskJacobian( iCub::iKin::iKinChain *right_Arm_Chain, 
                                              iCub::iKin::iKinChain *LeftLegChain,
                                              iCub::iKin::iKinChain *RightLegChain,
                                              bool StanceLeg);



	// Grasping Jacobian of the left hand wrt. the General stance leg
	MatrixXd getGraspJacobianLhand_GenStanceLeg(  MatrixXd d_H_c,
                                                iCub::iKin::iKinChain *left_Arm_Chain, 
                                                  iCub::iKin::iKinChain *LeftLegChain,
                                                  iCub::iKin::iKinChain *RightLegChain,
                                                  bool StanceLeg);

	// Grasping Jacobian of the right hand wrt. the General stance leg
	MatrixXd getGraspJacobianRhand_GenStanceLeg(  MatrixXd d_H_c,
                                                iCub::iKin::iKinChain *right_Arm_Chain, 
                                                  iCub::iKin::iKinChain *LeftLegChain,
                                                  iCub::iKin::iKinChain *RightLegChain,
                                                  bool StanceLeg);

	// left hand joint velocities for the grasping task  (with Gen stance leg as world frame)
	VectorXd get_lh_GraspJtsVelocity_GStance( MatrixXd dl_H_cl,
                                            iCub::iKin::iKinChain *left_Arm_Chain,
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            Eigen::VectorXd q_dot_lleg,
                                            Eigen::VectorXd q_dot_rleg,
                                            bool StanceLeg);

	// right hand joint velocities for the grasping task  (with Gen stance leg as world frame) 
	VectorXd get_rh_GraspJtsVelocity_GStance( MatrixXd dr_H_cr,
                                            iCub::iKin::iKinChain *right_Arm_Chain,
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            Eigen::VectorXd q_dot_lleg,
                                            Eigen::VectorXd q_dot_rleg,
                                            bool StanceLeg);



	// kinematics transformations with respect to the world frame (as measured)
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// Velocity twist matrix of the wolrd frame wrt. the left hand frame (h_W_w)
	MatrixXd getLhand_World_VeloTwistMx( MatrixXd wld_Trsf_root,
										                   iCub::iKin::iKinChain *left_Arm_Chain);

	// Velocity twist matrix of a General stance foot frame (taken as the wolrd frame) wrt. the right hand frame (h_W_w)
	MatrixXd getRhand_World_VeloTwistMx( MatrixXd wld_Trsf_root,
										                   iCub::iKin::iKinChain *right_Arm_Chain);

	// Task Jacobian of the left hand associated to the robot root link with left stance leg
	MatrixXd getLhand_VBase_TskJac_World( MatrixXd wld_Trsf_root,
										                    iCub::iKin::iKinChain *left_Arm_Chain);

	// Task Jacobian of the left hand associated to the robot root link with left stance leg
	MatrixXd getRhand_VBase_TskJac_World( MatrixXd wld_Trsf_root,
										                    iCub::iKin::iKinChain *right_Arm_Chain);


	// Task Jacobian of left hand wrt. the stance legs and all expressed in the world frame  ( think about writing this as function of virtual base)
	MatrixXd getLhand_world_TaskJacobian( MatrixXd wld_Trsf_root,
                                        iCub::iKin::iKinChain *left_Arm_Chain, 
                                        iCub::iKin::iKinChain *LeftLegChain,
                                        iCub::iKin::iKinChain *RightLegChain,
                                        bool StanceLeg);

	// Task jacobian of right hand wrt. the stance legs and all expressed in the world frame
	MatrixXd getRhand_world_TaskJacobian( MatrixXd wld_Trsf_root,
                                        iCub::iKin::iKinChain *right_Arm_Chain, 
                                        iCub::iKin::iKinChain *LeftLegChain,
                                        iCub::iKin::iKinChain *RightLegChain,
                                        bool StanceLeg);


  // Grasping Jacobian of the left hand wrt. the General stance leg
  MatrixXd getGraspJacobianLhand_World( MatrixXd wld_Trsf_root,
                                        MatrixXd dl_H_cl,
                                        iCub::iKin::iKinChain *left_Arm_Chain, 
                                        iCub::iKin::iKinChain *LeftLegChain,
                                        iCub::iKin::iKinChain *RightLegChain,
                                        bool StanceLeg);

  // Grasping Jacobian of the right hand wrt. the General stance leg
  MatrixXd getGraspJacobianRhand_World( MatrixXd wld_Trsf_root,
                                        MatrixXd dr_H_cr,
                                        iCub::iKin::iKinChain *right_Arm_Chain, 
                                        iCub::iKin::iKinChain *LeftLegChain,
                                        iCub::iKin::iKinChain *RightLegChain,
                                        bool StanceLeg);


	// left hand joint velocities for the grasping task  wrt. the world frame 
	VectorXd get_lh_GraspJtsVelocity_World( MatrixXd wld_Trsf_root,
											                    MatrixXd dl_H_cl,
                                          iCub::iKin::iKinChain *left_Arm_Chain,
                                          iCub::iKin::iKinChain *LeftLegChain,
                                          iCub::iKin::iKinChain *RightLegChain,
                                          Eigen::VectorXd q_dot_lleg,
                                          Eigen::VectorXd q_dot_rleg,
                                          bool StanceLeg);


	// right hand joint velocities for the grasping task  wrt. the world frame
	VectorXd get_rh_GraspJtsVelocity_World( MatrixXd wld_Trsf_root,
											                    MatrixXd dr_H_cr,    
                                          iCub::iKin::iKinChain *right_Arm_Chain,
                                          iCub::iKin::iKinChain *LeftLegChain,
                                          iCub::iKin::iKinChain *RightLegChain,
                                          Eigen::VectorXd q_dot_lleg,
                                          Eigen::VectorXd q_dot_rleg,
                                          bool StanceLeg);

  // left hand joint velocities for the grasping task  wrt. the world frame 
  VectorXd get_lh_GraspJtsVelocity_World2( MatrixXd wld_Trsf_root,
                                          MatrixXd dl_H_cl,
                                          iCub::iKin::iKinChain *left_Arm_Chain,
                                          iCub::iKin::iKinChain *LeftLegChain,
                                          iCub::iKin::iKinChain *RightLegChain,
                                          Eigen::VectorXd q_dot_lleg,
                                          Eigen::VectorXd q_dot_rleg,
                                          bool StanceLeg);


  // right hand joint velocities for the grasping task  wrt. the world frame 
  VectorXd get_rh_GraspJtsVelocity_World2(  MatrixXd wld_Trsf_root,
                                            MatrixXd dr_H_cr,
                                            iCub::iKin::iKinChain *right_Arm_Chain,
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            Eigen::VectorXd q_dot_lleg,
                                            Eigen::VectorXd q_dot_rleg,
                                            bool StanceLeg); 


    // bimanual joint velocities for the grasping task  wrt. the world frame
	void get_bimanual_GraspJtsVelocity_World( MatrixXd wld_Trsf_root,
											                      MatrixXd dl_H_cl,
                                            MatrixXd dr_H_cr,  
                                            iCub::iKin::iKinChain *left_Arm_Chain,
                                            iCub::iKin::iKinChain *right_Arm_Chain, 
                                            iCub::iKin::iKinChain *LeftLegChain,
                                            iCub::iKin::iKinChain *RightLegChain,
                                            Eigen::VectorXd q_dot_lleg,
                                            Eigen::VectorXd q_dot_rleg,
                                            bool StanceLeg); 
  VectorXd getLeftHand_JointsLimitsGradient( iCub::iKin::iKinChain *left_Arm_Chain, double rho);

  VectorXd getRightHand_JointsLimitsGradient( iCub::iKin::iKinChain *right_Arm_Chain, double rho);

  void set_lh_GraspGain(VectorXd Gain_lh);

  void set_rh_GraspGain(VectorXd Gain_rh);


  // left hand joint Torques for the grasping task  wrt. the world frame 
  VectorXd get_lh_GraspJtsTorques( MatrixXd wld_Trsf_root,
                                   MatrixXd dl_H_cl,
                                   iCub::iKin::iKinChain *left_Arm_Chain,
                                   iCub::iKin::iKinChain *LeftLegChain,
                                   iCub::iKin::iKinChain *RightLegChain,
                                   Eigen::VectorXd q_dot_lleg,
                                   Eigen::VectorXd q_dot_rleg,
                                   bool StanceLeg);


  // right hand joint Torques for the grasping task  wrt. the world frame 
  VectorXd get_rh_GraspJtsTorques(  MatrixXd wld_Trsf_root,
                                    MatrixXd dr_H_cr,
                                    iCub::iKin::iKinChain *right_Arm_Chain,
                                    iCub::iKin::iKinChain *LeftLegChain,
                                    iCub::iKin::iKinChain *RightLegChain,
                                    Eigen::VectorXd q_dot_lleg,
                                    Eigen::VectorXd q_dot_rleg,
                                    bool StanceLeg); 


// left hand joint Torques for the grasping task  wrt. the world frame 
  VectorXd get_lh_GraspJtsTorquesGlobal( MatrixXd wld_Trsf_root,
                                         MatrixXd dl_H_cl,
                                         iCub::iKin::iKinChain *left_Arm_Chain,
                                         iCub::iKin::iKinChain *LeftLegChain,
                                         iCub::iKin::iKinChain *RightLegChain,
                                         Eigen::VectorXd q_dot_lleg,
                                         Eigen::VectorXd q_dot_rleg,
                                         bool StanceLeg);


  // right hand joint Torques for the grasping task  wrt. the world frame 
  VectorXd get_rh_GraspJtsTorquesGlobal(  MatrixXd wld_Trsf_root,
                                          MatrixXd dr_H_cr,
                                          iCub::iKin::iKinChain *right_Arm_Chain,
                                          iCub::iKin::iKinChain *LeftLegChain,
                                          iCub::iKin::iKinChain *RightLegChain,
                                          Eigen::VectorXd q_dot_lleg,
                                          Eigen::VectorXd q_dot_rleg,
                                          bool StanceLeg); 

  // left hand joints velocity from admittance
  VectorXd getLArmJointsVelocityFromAdmittance( MatrixXd wld_Trsf_root,
                                                MatrixXd dl_H_cl,
                                                iCub::iKin::iKinChain *left_Arm_Chain,
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                Eigen::VectorXd q_dot_lleg,
                                                Eigen::VectorXd q_dot_rleg,
                                                bool StanceLeg,
                                                Eigen::VectorXd lhandAppWrench);


  // right hand joints velocity from admittance
  VectorXd getRArmJointsVelocityFromAdmittance( MatrixXd wld_Trsf_root,
                                                MatrixXd dr_H_cr,
                                                iCub::iKin::iKinChain *right_Arm_Chain,
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                Eigen::VectorXd q_dot_lleg,
                                                Eigen::VectorXd q_dot_rleg,
                                                bool StanceLeg,
                                                Eigen::VectorXd rhandAppWrench); 

  // void setVirtualInertia(VectorXd l_inertia, VectorXd right_Inertia);
  // void setVirtualDamping(VectorXd l_damping, VectorXd right_damping);
  // void setVirtualStiffness(VectorXd l_stiffness, VectorXd right_stiffness);
  bool InitializeLeftAdmittance(double SamplingTime, MatrixXd DynM, MatrixXd ControlMx, VectorXd Input);

  bool InitializeRightAdmittance(double SamplingTime, MatrixXd DynM, MatrixXd ControlMx, VectorXd Input);

    // Dynamic system solver for the admittance law
    DynamicSystemSolver2  left_hand_admittance;
    DynamicSystemSolver2  right_hand_admittance;

    // Properties of the admittance law
    VectorXd lh_virtualInertia;
    VectorXd rh_virtualInertia;

    VectorXd lh_virtualDamping;
    VectorXd rh_virtualDamping;

    VectorXd lh_virtualStiffness;
    VectorXd rh_virtualStiffness;

  void setVirtualStiffness(VectorXd l_stiffness, VectorXd r_stiffness);

  bool UpdateTaskTransformations( MatrixXd world_H_root,
                                            yarp::sig::Vector Des_lhandPose_in_ref,   // reference frame could be the root or the world frame
                                            yarp::sig::Vector Des_rhandPose_in_ref,
                                            RobotKinematics &botKin,
                                            bool useWorldRef);               // if true, the reference frame is the world frame otherwise
                                                                   // the root frame is the reference frame


//
//
Eigen::VectorXd Weight_qDot;

Eigen::MatrixXd Gsp_Hessian_matrix_lhand;
Eigen::VectorXd Gsp_Gradient_vector_lhand;
Eigen::MatrixXd Gsp_Hessian_matrix_rhand;
Eigen::VectorXd Gsp_Gradient_vector_rhand;

Eigen::MatrixXd Inequality_matrix_lhand;
Eigen::VectorXd Inequality_vector_lhand;
Eigen::MatrixXd Inequality_matrix_rhand;
Eigen::VectorXd Inequality_vector_rhand;

Eigen::VectorXd velocity_Saturation_Limit;


// left hand joint velocities for the grasping task  wrt. the world frame 
bool get_lh_GraspWorld2_HessianGradient(  MatrixXd wld_Trsf_root,
                                          MatrixXd dl_H_cl,
                                          iCub::iKin::iKinChain *left_Arm_Chain,
                                          iCub::iKin::iKinChain *LeftLegChain,
                                          iCub::iKin::iKinChain *RightLegChain,
                                          Eigen::VectorXd q_dot_lleg,
                                          Eigen::VectorXd q_dot_rleg,
                                          bool StanceLeg);

// right hand joint velocities for the grasping task  wrt. the world frame 
bool get_rh_GraspWorld2_HessianGradient(  MatrixXd wld_Trsf_root,
                                          MatrixXd dr_H_cr,
                                          iCub::iKin::iKinChain *right_Arm_Chain,
                                          iCub::iKin::iKinChain *LeftLegChain,
                                          iCub::iKin::iKinChain *RightLegChain,
                                          Eigen::VectorXd q_dot_lleg,
                                          Eigen::VectorXd q_dot_rleg,
                                          bool StanceLeg) ;

//
bool get_lhand_InequalityConstraints( double SamplingTime,
                                      iCub::iKin::iKinChain *left_Arm_Chain,
                                      Eigen::VectorXd velocitySaturationLimit);

//
bool get_rhand_InequalityConstraints( double SamplingTime,
                                      iCub::iKin::iKinChain *right_Arm_Chain,
                                      Eigen::VectorXd velocitySaturationLimit);

// ====================================== INITIALIZATION OF QP SOLVER ========================================================
// QP oases solver
QPOasesSolver               *qpOSolver_lhand;
// QP oases solver
QPOasesSolver               *qpOSolver_rhand;

// left hand joint velocities for the grasping task  wrt. the world frame 
bool InitializeQPsolver_lh_GraspWorld2(  MatrixXd wld_Trsf_root,
                                        MatrixXd dl_H_cl,
                                        iCub::iKin::iKinChain *left_Arm_Chain,
                                        iCub::iKin::iKinChain *LeftLegChain,
                                        iCub::iKin::iKinChain *RightLegChain,
                                        Eigen::VectorXd q_dot_lleg,
                                        Eigen::VectorXd q_dot_rleg,
                                        bool StanceLeg,
                                        double SamplingTime,
                                        Eigen::VectorXd velocitySaturationLimit);

// right hand joint velocities for the grasping task  wrt. the world frame 
bool InitializeQPsolver_rh_GraspWorld2(  MatrixXd wld_Trsf_root,
                                        MatrixXd dr_H_cr,
                                        iCub::iKin::iKinChain *right_Arm_Chain,
                                        iCub::iKin::iKinChain *LeftLegChain,
                                        iCub::iKin::iKinChain *RightLegChain,
                                        Eigen::VectorXd q_dot_lleg,
                                        Eigen::VectorXd q_dot_rleg,
                                        bool StanceLeg,
                                        double SamplingTime,
                                        Eigen::VectorXd velocitySaturationLimit);

// ====================================== GET QP SOLUTION ========================================================

// left hand joint velocities for the grasping task  wrt. the world frame 
Eigen::VectorXd GetQPsolution_lh_GraspWorld2(   MatrixXd wld_Trsf_root,
                                                MatrixXd dl_H_cl,
                                                iCub::iKin::iKinChain *left_Arm_Chain,
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                Eigen::VectorXd q_dot_lleg,
                                                Eigen::VectorXd q_dot_rleg,
                                                bool StanceLeg,
                                                double SamplingTime,
                                                Eigen::VectorXd velocitySaturationLimit);


// right hand joint velocities for the grasping task  wrt. the world frame 
Eigen::VectorXd GetQPsolution_rh_GraspWorld2(   MatrixXd wld_Trsf_root,
                                                MatrixXd dr_H_cr,
                                                iCub::iKin::iKinChain *right_Arm_Chain,
                                                iCub::iKin::iKinChain *LeftLegChain,
                                                iCub::iKin::iKinChain *RightLegChain,
                                                Eigen::VectorXd q_dot_lleg,
                                                Eigen::VectorXd q_dot_rleg,
                                                bool StanceLeg,
                                                double SamplingTime,
                                                Eigen::VectorXd velocitySaturationLimit);


};



#endif // Grasping_H