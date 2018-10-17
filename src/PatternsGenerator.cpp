

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


// PatternsGenerator
// 	10.5. CP_SelectionMatrices
// 	10.2. CpStanceFootPose
// 	10.6. VelocitiesSetPoints
// 	10.1. CpReactiveWalkingController
// 	10.3. CpFootTrajectories
// 	10.4. CpGaitTransformations


#include <string>
#include <iostream>
#include <fstream>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "PatternsGenerator.h"

//using namespace std;
//using namespace Eigen;





// ====================================================================================================================



// ====================================================================================================================

CpFootTrajectories::CpFootTrajectories(int SptFt[],
                                       VectorXd RelCoPXYR,
                                       CpStanceFootPose *CoPRef,
                                       CP_SelectionMatrices *SMx,
                                                   double FtZmax)

{
    // resize the vector PrevStanceXY
    PrevStanceXY.resize(2);
    PrevStanceXY.setZero(2);
    // Assigning foot maximun height
    MaxFootHeight  = FtZmax;

    // stance foot indicator
    StanceIndicator[0] = SptFt[0];
    StanceIndicator[1] = SptFt[1];

    // calling the FooTrajectories methodes
    CpFootTrajectories::ComputeFootTrajectories(    SptFt,
                                                    RelCoPXYR,
                                                    CoPRef,
                                                        SMx);
                       
}
                       
void CpFootTrajectories::ComputeFootTrajectories(              int SptFt[],
                                                        VectorXd RelCoPXYR,
                                                 CpStanceFootPose *CoPRef,
                                                 CP_SelectionMatrices *SMx)
                        
{
    //********************************************************
    // update the stance foot states
  // stance foot indicator
    StanceIndicator[0] = SptFt[0];
    StanceIndicator[1] = SptFt[1];
    // Input Arguments
                    //T           = 0.1;
    tm = SMx->IndexSFt * ((SMx->DrS[0]/SMx->Tp)/(SMx->DrS[0]/SMx->Tp -1));
    double Ts = SMx->DrS[0]/SMx->Tp;

   //********************************************************
   // some properties of the class
   PrevStanceXY   = CoPRef->xk_fc_xy_n1;
   PrevStanceAng  = CoPRef->xk_fc_ang_n1;

    double z_max = MaxFootHeight;
   //
    double D_theta, D_theta_p1;
   // Delta_theta (Theta_0 - Theta_-1)
    D_theta      = CoPRef->CoPRefR - PrevStanceAng;
   //Delta_theta+1 (Theta_+1 -Theta_0)
    //D_theta_p1   = (QPSol.OrienCoP)(0) - CoPRef->CoPRefR;

    D_theta_p1   = RelCoPXYR(8) - CoPRef->CoPRefR;  // RelCoPXYR(8)== OrienCoP(0)

   // angle for COM orientation
    double Angtj_ts;
    Angtj_ts = -2*((D_theta)/pow(Ts,3.))*pow(tm,3.) +
                   3*(D_theta/pow(Ts,2.))*pow(tm,2.);

   //%%%%%%%% X, Y, Z and Theta TRAJECTORIES %%%%%%%%%%%%%%%
   //
   // Computation of previous and future footstep relative to the
   // stance foot
   // ----------------------------------------------------------
            
   // Previous and 1st future steps relative to stance foot
   // sxyn1 and sxyp1
   sxyn1.resize(2); // previous step relative to stance (sxy1)
   sxyp1.resize(2); // 1st future step relative to stance (sxy2_t)
                             // as funcion of time (at every sample instant)

   sxyn1(0) =  cos(D_theta)*PrevStanceXY(0) + sin(D_theta)*PrevStanceXY(1);
   sxyn1(1) = -sin(D_theta)*PrevStanceXY(0) + cos(D_theta)*PrevStanceXY(1);

   sxyp1(0) = (1.+cos(-D_theta+Angtj_ts))*1/2.*(RelCoPXYR)(0)    //RelCoPXYR(0) == RelCoPXY(0)
             - sin(-D_theta+Angtj_ts)*1/2.*(RelCoPXYR)(1);       //RelCoPXYR(1) == RelCoPXY(1)
   sxyp1(1) = sin(-D_theta+Angtj_ts)*1/2.*(RelCoPXYR)(0)
             + (1.+cos(-D_theta+Angtj_ts))*1/2.*(RelCoPXYR)(1);

   /* ********************************************************
    * Cubic polynomial trajectory for X and Y
   //-----------------------------------------
   //
      double xtj_ts, //
             ytj_ts;
            xtj_ts = -2*((sxyp1(1,1)-sxyn1(1,1))/pow(Ts, 3))*pow(tm,3 +...
                      3*((sxyp1(1,1)-sxyn1(1,1))/pow(Ts, 2))*pow(tm,2 +...
                      sxyn1(1,1);
   //
         ytj_ts = -2*((sxyp1(2,1)-sxyn1(2,1))/pow(Ts, 3))*pow(tm,3 +...
                   3*((sxyp1(2,1)-sxyn1(2,1))/pow(Ts, 2))*pow(tm,2 +...
                   sxyn1(2,1);
   //
   // polynomial trajectory for Z via zmax
   //-------------------------------------
        ztj_ts = 16*(z_max/pow(Ts, 4))*pow(tm,4 -...
                 32*(z_max/pow(Ts, 3))*pow(tm,3 +...
                 16*(z_max/pow(Ts, 2))*pow(tm,2;
                     *
   // Angle for swinging foot orientation
   //--------------------------------------
      A_mft_tj_ts = -2*((D_theta_p1+D_theta)/pow(Ts, 3))*pow(tm,3 +...
                     3*((D_theta_p1+D_theta)/pow(Ts, 2))*pow(tm,2 -...
                        D_theta;
    * *****************************************************  */

    // Quintic Polynomial trajectories for X and Y
    // ===========================================
    double xtj_ts, // x swing foot trajectory
           ytj_ts, // y swing trajectory
           ztj_ts, // z swing trajectory
           A_mft_tj_ts; // angular swing trajectory
           
           xtj_ts =   6*((sxyp1(0)-sxyn1(0))/pow(Ts,5.))*pow(tm,5.)
                    -15*((sxyp1(0)-sxyn1(0))/pow(Ts,4.))*pow(tm,4.)
                    +10*((sxyp1(0)-sxyn1(0))/pow(Ts,3.))*pow(tm,3.)
                    + sxyn1(0);
                     
           ytj_ts =   6*((sxyp1(1)-sxyn1(1))/pow(Ts,5.))*pow(tm,5.)
                    -15*((sxyp1(1)-sxyn1(1))/pow(Ts,4.))*pow(tm,4.)
                    +10*((sxyp1(1)-sxyn1(1))/pow(Ts,3.))*pow(tm,3.)
                     + sxyn1(1);
                     
            // polynomial trajectory for Z via zmax
            //-------------------------------------
            ztj_ts = - 64*(z_max/pow(Ts,6.))*pow(tm,6.)
                     +192*(z_max/pow(Ts,5.))*pow(tm,5.)
                     -192*(z_max/pow(Ts,4.))*pow(tm,4.)
                     + 64*(z_max/pow(Ts,3.))*pow(tm,3.);

//            //-------------------------------------
//                 ztj_ts =  16*(z_max/pow(Ts, 4.))*pow(tm,4.)
//                          -32*(z_max/pow(Ts, 3.))*pow(tm,3.)
//                          +16*(z_max/pow(Ts, 2.))*pow(tm,2.);
  
            // Angle for swinging foot orientation
            //--------------------------------------
            A_mft_tj_ts =  6*((D_theta_p1+D_theta)/pow(Ts,5.))*pow(tm,5.)
                         -15*((D_theta_p1+D_theta)/pow(Ts,4.))*pow(tm,4.)
                         +10*((D_theta_p1+D_theta)/pow(Ts,3.))*pow(tm,3.)
                         - D_theta;  // -ve sign
    //
    // Feet trajectories in Stance Foot Frame
    // =======================================
    // Left Foot
       XTrajLeftFoot   = xtj_ts * SptFt[1]; //Uc_k_0(1,1);         // X
       YTrajLeftFoot   = ytj_ts * SptFt[1]; //Uc_k_0(1,1);         // Y
       ZTrajLeftFoot   = ztj_ts * SptFt[1]; //Uc_k_0(1,1);         // Z
       AngTrajLeftFoot = A_mft_tj_ts * SptFt[1]; //Uc_k_0(1,1);    // theta
    //
    // Right Foot
       XTrajRightFoot   = xtj_ts * SptFt[0]; //Uk_0(1,1);         // X
       YTrajRightFoot   = ytj_ts * SptFt[0]; //Uk_0(1,1);         // Y
       ZTrajRightFoot   = ztj_ts * SptFt[0]; //Uk_0(1,1);         // Z
       AngTrajRightFoot = A_mft_tj_ts * SptFt[0]; //Uk_0(1,1);    // theta
    //
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //
       DelTheta0n1 = D_theta;
       DelThetap1  = D_theta_p1;
                      
    //
}
    
void CpFootTrajectories::SetMaxFootHeight(double z_max)
{
  MaxFootHeight = z_max;  
};


// ====================================================================================================================



// ====================================================================================================================

CpGaitTransformations::CpGaitTransformations(Discrete_CP_LIP_Model *St,
                              int SptFt[],
                                              CpStanceFootPose *CoPref,
                                             CpFootTrajectories *FtTraj,
                                             VectorXd t_B_CoM)
{
  // translation from Com to base %%% (base expressed in com frame)
  TrslBaseCoM = t_B_CoM; //[x_B_com; y_B_com; z_B_com];
  
    CpGaitTransformations:: ComputeGaitTransforms(     St,
                          SptFt,
                           CoPref,
                            FtTraj);
                        
                         
}
  
void CpGaitTransformations:: ComputeGaitTransforms( Discrete_CP_LIP_Model *St,
                                   int SptFt[],
                                                     CpStanceFootPose *CoPref,
                                                    CpFootTrajectories *FtTraj)
{
            
    

    
    // From inertial to support foot frame %%%
    // Transformation from support for to inertial frame
        
    MatrixXd T_sft_inertial_f, // Homogeneous transformation from 
           R_sft_inertial_f; // Rotation matrix    
    VectorXd trsl_sft_inertial_f(3); // Translation vector
    // Rotation matrix from sft to inertial from
    R_sft_inertial_f.resize(3,3);
    R_sft_inertial_f.setZero(3,3);
        
    R_sft_inertial_f(0,0) =  cos(CoPref->CoPRefR);
    R_sft_inertial_f(0,1) = -sin(CoPref->CoPRefR);
    R_sft_inertial_f(1,0) =  sin(CoPref->CoPRefR);
    R_sft_inertial_f(1,1) =  cos(CoPref->CoPRefR);
    R_sft_inertial_f(2,2) =  1.;
  // Translation vector from sft to inertial frame
    trsl_sft_inertial_f(0) = CoPref->CoPRefX;  //ZMP ref x
    trsl_sft_inertial_f(1) = CoPref->CoPRefY;  //ZMP ref y
    trsl_sft_inertial_f(2) = 0.;        //ZMP height
    
  // Homogeneous Transformation from sft to inertial frame
    T_sft_inertial_f.resize(4,4); T_sft_inertial_f.setZero(4,4);
    T_sft_inertial_f(3,3) = 1;
  
    T_sft_inertial_f.block(0, 0, 3, 3) = R_sft_inertial_f;
    T_sft_inertial_f.block(0, 3, 3, 1) = trsl_sft_inertial_f;
    
  //  Transformation from CoM to Support foot
  // position of CoM in support foot 
  VectorXd trsl_com_sft_inertial(3), 
        trsl_com_sft(3); 
      
  // expressed in the inertial frame
    trsl_com_sft_inertial(0) = (St->StatesX)(0) - CoPref->CoPRefX;
    trsl_com_sft_inertial(1) = (St->StatesY)(0) - CoPref->CoPRefY;
    trsl_com_sft_inertial(2) = St->zc;
    
  // expressed in the support ft frame 
    trsl_com_sft = R_sft_inertial_f.transpose() * trsl_com_sft_inertial;

    // Homogeneous tranformation of Feet trajectories in the base 
        // transformation from base to support foot
        MatrixXd T_B_sft, Eye_3;

        Eye_3.resize(3,3);
        Eye_3.setIdentity(3,3);
        T_B_sft.resize(4,4); T_B_sft.setZero(4,4);
        T_B_sft(3,3) = 1;
        T_B_sft.block(0, 0, 3, 3) =  Eye_3;

    // Adapting transformation from General Inertial frame to Icub definition frame
       /* The general inertial frame is defined with the x axis lying in the frontal plane
        * pointing in front of the robot (roll axis)
        * the y axis form the center to the left foot and the z axis in the vertical direction
        * opposite to the gravity
        * the general base frame follows the same convention */
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

    // Transformation from icub foot frame to the Genaral foot frame
        MatrixXd T_icubF_GF, R_icubF_GF, invT_icubF_GF;
        T_icubF_GF.resize(4,4);
        T_icubF_GF.setZero(4,4);   T_icubF_GF(3,3) = 1.;

        R_icubF_GF.resize(3,3);
        R_icubF_GF.setZero(3,3);
        R_icubF_GF(0,2) =  1.0;
        R_icubF_GF(1,1) = -1.0;
        R_icubF_GF(2,0) =  1.0;

        T_icubF_GF.block(0,0, 3, 3) = R_icubF_GF;

        // inverse transformation From icub Base to the General Base
        invT_icubF_GF.resize(4,4);
        invT_icubF_GF.setZero(4,4);   invT_icubF_GF(3,3) = 1.;

        invT_icubF_GF.block(0,0,3,3) = R_icubF_GF.transpose();


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        MatrixXd T_Pitch_Base;
        T_Pitch_Base.resize(4,4); T_Pitch_Base.setZero(4,4); T_Pitch_Base(3,3) = 1.;
        double pitch;
        pitch = 1. *0.00; //0.03
        T_Pitch_Base(0,0) = cos(pitch);
        T_Pitch_Base(0,2) = sin(pitch);
        T_Pitch_Base(1,1) = 1.;
        T_Pitch_Base(2,0) = -sin(pitch);
        T_Pitch_Base(2,2) = cos(pitch);

    // feet roll corection
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
        double roll_coef = 1.0 *0.0;  // 0.03
        MatrixXd T_l_foot_roll(4,4),
                 T_r_foot_roll(4,4);

        T_l_foot_roll.setZero(4,4);     T_l_foot_roll(3,3) = 1.;
        T_r_foot_roll.setZero(4,4);     T_r_foot_roll(3,3) = 1.;
        // left
        T_l_foot_roll(0,0) =  cos(roll_coef);
        T_l_foot_roll(0,1) = -sin(roll_coef);
        T_l_foot_roll(1,0) =  sin(roll_coef);
        T_l_foot_roll(1,1) =  cos(roll_coef);
        T_l_foot_roll(2,2) =  1.;
        // right
        T_r_foot_roll(0,0) =  cos(roll_coef);  //-
        T_r_foot_roll(0,1) = -sin(roll_coef);
        T_r_foot_roll(1,0) =  sin(roll_coef);
        T_r_foot_roll(1,1) =  cos(roll_coef);
        T_r_foot_roll(2,2) =  1.;


    // **********************************************************************


    if  (SptFt[0] ==1)  // the left foot is the stance foot
    {
            
    // Transformation from the CoM to the left stance foot
      
    MatrixXd T_com_sft, // Homogeneous transformation
           R_com_sft; // Rotation matrix
      
    // Rotation matrix from CoM to Sft
    R_com_sft.resize(3,3);
        R_com_sft.setZero(3,3);
    
        R_com_sft(0,0) =  cos(FtTraj->AngTrajRightFoot/2.);
        R_com_sft(0,1) = -sin(FtTraj->AngTrajRightFoot/2.);
        R_com_sft(1,0) =  sin(FtTraj->AngTrajRightFoot/2.);
        R_com_sft(1,1) =  cos(FtTraj->AngTrajRightFoot/2.);
        R_com_sft(2,2) =  1.;
      
    // Homogeneous transformation from CoM to Sft
        T_com_sft.resize(4,4);  T_com_sft.setZero(4,4);  T_com_sft(3,3) = 1;
      
        T_com_sft.block(0, 0, 3, 3) = R_com_sft;
        T_com_sft.block(0, 3, 3, 1) = trsl_com_sft;
      
        // Homogeneous transformation from support foot to base
        MatrixXd  T_sft_B;
    /*T_B_com   = [[1 0 0;0 1 0;0 0 1] TrslBaseCoM; 0 0 0 1];

    //T_B_sft   = T_com_sft * T_B_com;
    * 
        * T_B_sft.resize(4,4); T_B_sft[3,3] = 1;
    * T_B_sft.insert(R_com_sft, 1, 1);
    * T_B_sft.insert(R_com_sft*TrslBaseCoM + trsl_com_sft, 1, 4);
    * T_sft_B   = inv(T_B_sft);    //
    * */


        T_B_sft.block(0, 0, 3, 3) = R_com_sft;
        T_B_sft.block(0, 3, 3, 1) = R_com_sft*TrslBaseCoM + trsl_com_sft;

 //         cout << "T_B_sft is : \n"<< R_com_sft*TrslBaseCoM << endl;
      
        T_sft_B.resize(4,4); T_sft_B.setZero(4,4); T_sft_B(3,3) = 1;
      
        T_sft_B.block(0, 0, 3, 3) =  R_com_sft.transpose();
        T_sft_B.block(0, 3, 3, 1) = -TrslBaseCoM - R_com_sft.transpose()*trsl_com_sft;

        // Transformation from right moving (swing) foot to Support foot
        MatrixXd T_right_mft_sft,   // Homogeneous transformation
           R_right_mft_sft;   // Rotation matrix
    VectorXd trsl_right_mft_sft(3); // Translation vector
      
    // Rotation matrix from right swing ft to L sft
    R_right_mft_sft.resize(3,3);
        R_right_mft_sft.setZero(3,3);
      
        R_right_mft_sft(0,0) =  cos(FtTraj->AngTrajRightFoot);
        R_right_mft_sft(0,1) = -sin(FtTraj->AngTrajRightFoot);
        R_right_mft_sft(1,0) =  sin(FtTraj->AngTrajRightFoot);
        R_right_mft_sft(1,1) =  cos(FtTraj->AngTrajRightFoot);
        R_right_mft_sft(2,2) =  1.;
      
    // Translation vector from R moving ft to L stance foot
        trsl_right_mft_sft(0) = FtTraj->XTrajRightFoot;
        trsl_right_mft_sft(1) = FtTraj->YTrajRightFoot;
        trsl_right_mft_sft(2) = FtTraj->ZTrajRightFoot;
      
    // Homogeneous Transformation from R moving ft to L stance foot
        T_right_mft_sft.resize(4,4);  T_right_mft_sft.setZero(4,4);
        T_right_mft_sft(3,3) = 1;
      
        T_right_mft_sft.block(0, 0, 3, 3) = R_right_mft_sft;
        T_right_mft_sft.block(0, 3, 3, 1) = trsl_right_mft_sft;
      
    // Overall Transformations
    // **************************


    // Transformation from Left Leg to the Base Frame
           TrsfLeftLegBase  = T_Pitch_Base * T_GB_icubB * T_sft_B * T_icubF_GF;
    // Transformation from Right Leg to the Base Frame
           TrsfRightLegBase = T_Pitch_Base * T_GB_icubB * T_sft_B * T_right_mft_sft  * T_icubF_GF * T_r_foot_roll;
        //
      }
            
      else   // (Uc_k_0(1,1)==1) the right foot is the stance foot
      {


    // Rotation from the CoM to the right stance foot
    MatrixXd T_com_sft, // Homogeneous transformation
         R_com_sft; // Rotation matrix
      
      // Rotation matrix
      R_com_sft.resize(3,3);
            R_com_sft.setZero(3,3);
      
            R_com_sft(0,0) =  cos(FtTraj->AngTrajLeftFoot/2.);
            R_com_sft(0,1) = -sin(FtTraj->AngTrajLeftFoot/2.);
            R_com_sft(1,0) =  sin(FtTraj->AngTrajLeftFoot/2.);
            R_com_sft(1,1) =  cos(FtTraj->AngTrajLeftFoot/2.);
            R_com_sft(2,2) =  1.;

//   cout << "AngTrajLeftFoot is : \n"<< FtTraj->AngTrajLeftFoot/2. << endl;
      
      // Homogeneous transformation
            T_com_sft.resize(4,4);  T_com_sft.setZero(4,4);
            T_com_sft(3,3) = 1;
      
            T_com_sft.block(0, 0, 3, 3) = R_com_sft;
            T_com_sft.block(0, 3, 3, 1) = trsl_com_sft;

            // Homogeneous transformation from support foot to base
            MatrixXd  T_sft_B;
      /*T_B_com   = [[1 0 0;0 1 0;0 0 1] TrslBaseCoM; 0 0 0 1];

      //T_B_sft   = T_com_sft * T_B_com;
      * 
            * T_B_sft.resize(4,4); T_B_sft[3,3] = 1;
      * T_B_sft.insert(R_com_sft, 1, 1);
      * T_B_sft.insert(R_com_sft*TrslBaseCoM + trsl_com_sft, 1, 4);
      * T_sft_B   = inv(T_B_sft);    //
      * */

            T_B_sft.block(0, 0, 3, 3) = R_com_sft;
            T_B_sft.block(0, 3, 3, 1) = R_com_sft*TrslBaseCoM + trsl_com_sft;

 //           cout << "T_B_sft is : \n"<< R_com_sft*TrslBaseCoM << endl;
      
            T_sft_B.resize(4,4); T_sft_B.setZero(4,4);
            T_sft_B(3,3) = 1;
      
            T_sft_B.block(0, 0, 3, 3) = R_com_sft.transpose();  // R_B_com = I
            T_sft_B.block(0, 3, 3, 1) = -TrslBaseCoM - R_com_sft.transpose()*trsl_com_sft;

            // Transformation from left moving (swing) foot to Support foot
            MatrixXd T_left_mft_sft,    // Homogeneous transformation
           R_left_mft_sft;    // Rotation matrix
      VectorXd trsl_left_mft_sft(3); // Translation vector
      
      // Rotation matrix from Left swing ft to R sft
            R_left_mft_sft.resize(3,3);     R_left_mft_sft.setZero(3,3);
      
            R_left_mft_sft(0,0) =  cos(FtTraj->AngTrajLeftFoot);
            R_left_mft_sft(0,1) = -sin(FtTraj->AngTrajLeftFoot);
            R_left_mft_sft(1,0) =  sin(FtTraj->AngTrajLeftFoot);
            R_left_mft_sft(1,1) =  cos(FtTraj->AngTrajLeftFoot);
            R_left_mft_sft(2,2) =  1.;
      
      // Translation vector from L moving ft to R stance foot
            trsl_left_mft_sft(0) = FtTraj->XTrajLeftFoot;
            trsl_left_mft_sft(1) = FtTraj->YTrajLeftFoot;
            trsl_left_mft_sft(2) = FtTraj->ZTrajLeftFoot;
      
      // Homogeneous Transformation from L moving ft to R stance foot
            T_left_mft_sft.resize(4,4);  T_left_mft_sft.setZero(4,4);
            T_left_mft_sft(3,3) = 1;
    
            T_left_mft_sft.block(0, 0, 3, 3) = R_left_mft_sft;
            T_left_mft_sft.block(0, 3, 3, 1) = trsl_left_mft_sft;
      
      // Overall Transformations
      // **************************
        // Transformation from Left Leg to the Base Frame
                TrsfLeftLegBase  = T_Pitch_Base * T_GB_icubB * T_sft_B * T_left_mft_sft  * T_icubF_GF * T_l_foot_roll;
        // Transformation from Right Leg to the Base Frame
                TrsfRightLegBase = T_Pitch_Base * T_GB_icubB * T_sft_B  * T_icubF_GF;
                //

            }
        //%% Homogeneous transformation of Base trajectory in World frame %%

        TrsfBaseInWorld = T_sft_inertial_f * T_B_sft;

        // Translation of the base relative to the stance foot frame.


        RefT_icubB_icubF = invT_icubF_GF * T_B_sft * invT_GB_icubB;

            // Desired positon of the icub base wrt. icub foot
            RefTransl_Base_in_Foot = RefT_icubB_icubF.block(0,3, 3, 1);
        
};

void CpGaitTransformations::SetTranslationBaseCoM(VectorXd t_B_CoM)
{
  // translation from Com to base %%% (base expressed in com frame)
  TrslBaseCoM = t_B_CoM;
};

// ====================================================================================



// ====================================================================================

// CpDesiredFeetTransformations::CpDesiredFeetTransformations()
// {

//     TrsfCurBaseInWorldIMU.resize(4,4);
//     TrsfCurBaseInWorldIMU.setIdentity(4,4);

//     FwKLeftFootInBase.resize(4,4);
//     FwKLeftFootInBase.setIdentity(4,4);;

//     FwKRightFootInBase.resize(4,4);
//     FwKRightFootInBase.setIdentity(4,4);
// }

// CpDesiredFeetTransformations::~CpDesiredFeetTransformations() {}

// void CpDesiredFeetTransformations::getDesiredFeetInCorrectedBase(int SptFt[],
//                                                                  yarp::sig::Vector LLegPose,
//                                                                  yarp::sig::Vector RLegPose,                                                               
//                                                                  yarp::sig::Vector m_orientation_rpy,
//                                                                  CpGaitTransformations *GTj)

// {

//     // Transformation from current foot frame in Horizontal base
//         // defining the left leg and right leg joints vectors
//         //Vector qLLin : left leg joints form encoders
//         //Vector qRLin : right joints angles from encoders
//         // yarp::sig::Vector LLegPose;
//         // yarp::sig::Vector RLegPose;

//         // // Create iCubLeg objects for the two legs
//         // iCub::iKin::iCubLeg limb_L_Leg("left_v2.5");   // left leg
//         // iCub::iKin::iCubLeg limb_R_Leg("right_v2.5");  // right leg

//         // // Create kinematic chains for the legs
//         // iCub::iKin::iKinChain *LeftLegChain;
//         // iCub::iKin::iKinChain *RightLegChain;   // MAY Be Should be declared as members of the class (TO ANALYSE)

//         // LeftLegChain  = limb_L_Leg.asChain();
//         // RightLegChain = limb_R_Leg.asChain();

//         // // assign the current leg joints values to the leg chains
//         // qLLin = LeftLegChain->setAng(qLLin);
//         // qRLin = RightLegChain->setAng(qRLin);

//         // // compute the current end-effector pose of the foot soles
//         // LLegPose = LeftLegChain->EndEffPose();
//         // RLegPose = RightLegChain->EndEffPose();

//         // Transformation from the Base to the IMU frame.
//         yarp::sig::Matrix Base2Imu;
//         yarp::sig::Vector rp_B2I;
//         rp_B2I = 0.1* m_orientation_rpy;
//         rp_B2I[2] = 0.0;

//         Base2Imu = yarp::math::rpy2dcm(CTRL_DEG2RAD * rp_B2I);
//         Eigen::VectorXd roll_pitch_Wld(3);
//         roll_pitch_Wld(0) = rp_B2I[0];
//         roll_pitch_Wld(1) = rp_B2I[1];
//         roll_pitch_Wld(2) = rp_B2I[2];

//         Eigen::MatrixXd Rot_Base2Imu = getComposedOrientFixedFrame(CTRL_DEG2RAD * roll_pitch_Wld);

//         // // extract values from yarp::Matrix and assign them to Eigen::MatrixXd
//         for (int row=0; row<3; row++)
//         {
//             for (int col=0; col<3; col++)
//             {
//                 TrsfCurBaseInWorldIMU(row,col) = Base2Imu(row,col);
//             }
//         }

       
//         // Building the 4X4 Homogeneous transformation matrix for the legs end effectors
//         FwKLeftFootInBase(0,3) = LLegPose[0];
//         FwKLeftFootInBase(1,3) = LLegPose[1];
//         FwKLeftFootInBase(2,3) = LLegPose[2];


//         FwKRightFootInBase(0,3) = RLegPose[0];
//         FwKRightFootInBase(1,3) = RLegPose[1];
//         FwKRightFootInBase(2,3) = RLegPose[2];

//         // Create a yarp matrix for the output of the yarp::math::axis2dcm method
//         yarp::sig::Matrix RotLLegEEf;
//         yarp::sig::Matrix RotRLegEEf;

//         RotLLegEEf = yarp::math::axis2dcm(LLegPose.subVector(3,6));
//         RotRLegEEf = yarp::math::axis2dcm(RLegPose.subVector(3,6));

//         // extract values from yarp::Matrix and assign them to Eigen::MatrixXd
//         for (int row=0; row<3; row++)
//         {
//             for (int col=0; col<3; col++)
//             {
//                 FwKLeftFootInBase(row,col) = RotLLegEEf(row,col);
//                 FwKRightFootInBase(row,col) = RotLLegEEf(row,col);
//             }
//         }


//         // Transformation from the IMU measurements  TrsfCurBaseInWorldIMU
//         //TrsfCurBaseInWorldIMU = ;

//         cout << " FwKLeftFootInBase is : \n" << FwKLeftFootInBase << endl;
//         cout << " FwKRightFootInBase is : \n" << FwKRightFootInBase << endl;



//         // Expressing the orientation of the world frame (IMU) in the base frame
//         MatrixXd TrsfWorldIMUInCurBase;

//         TrsfWorldIMUInCurBase.resize(4,4);
//         TrsfWorldIMUInCurBase.setZero(4,4);   TrsfWorldIMUInCurBase(3,3) = 1.;
//         TrsfWorldIMUInCurBase.block(0,0, 3,3) = (TrsfCurBaseInWorldIMU.block(0,0, 3,3)).transpose();

//         cout << " TrsfCurBaseInWorldIMU is :\n"<< TrsfCurBaseInWorldIMU << endl;


//         // Compute the current stance foot in the base frame
//         MatrixXd StanceFootInCurBase;

//         MatrixXd Rot_Base2BaseFoot(3,3);
//         Rot_Base2BaseFoot.setZero(3,3);
//         Rot_Base2BaseFoot(2,0) = -1.0;
//         Rot_Base2BaseFoot(1,1) =  1.0;
//         Rot_Base2BaseFoot(0,2) =  1.0;


//         if (SptFt[0] == 1)  // left stance foot
//         {

//             StanceFootInCurBase = FwKLeftFootInBase;
//             ReferenceTrsfHorFootInHorBase = GTj->TrsfLeftLegBase;
//             ReferenceTrsfSwingFootInHorBase = GTj->TrsfRightLegBase;

//             //
//             CurTrsfRealStanceFootInHorBase = TrsfCurBaseInWorldIMU * StanceFootInCurBase;                   // compute TrsfCurBaseInWorldIMU
//             // extracting the rotation matrix
//             CurRotRealStFootInHorBase = CurTrsfRealStanceFootInHorBase.block(0, 0, 3, 3);

//             // orientation of the current stance foot wrt. the horizontal plane
//             Vector3d EulerXYZ_F_hB, OrientXY_F_hF;
//             EulerXYZ_F_hB = getEulerAnglesXYZ_FixedFrame(Rot_Base2BaseFoot * CurRotRealStFootInHorBase);

//             // yarp::sig::Vector yEulerZYX_iF_iB(3);
//             // yEulerZYX_iF_iB = dcm2rpy(const yarp::sig::Matrix &R);
                    

//             OrientXY_F_hF    = EulerXYZ_F_hB;
//             OrientXY_F_hF(2) = 0.0;

//             RotRealStFootInHorStFoot = getComposedOrientFixedFrame(OrientXY_F_hF);
//             //EulerXYZ_F_hB = yarp::math::rpy2dcm(CTRL_DEG2RAD * OrientXY_F_hF);

//             // Desired Transformation of the real stance foot expressed in the Horizon base

//             // Trsf form real stance foot in horizontal stance foot frame
//             MatrixXd T_RealStFootInHorStFoot;

//             T_RealStFootInHorStFoot.resize(4,4);
//             T_RealStFootInHorStFoot.setZero(4,4);   T_RealStFootInHorStFoot(3,3) = 1.;
//             T_RealStFootInHorStFoot.block(0,0, 3,3) = RotRealStFootInHorStFoot;

//             cout << " EulerXYZ_F_hB is :\n"<< EulerXYZ_F_hB << endl;
//             cout << " T_RealStFootInHorStFoot is :\n"<< T_RealStFootInHorStFoot << endl;


//             DesTrsfRealStanceFootInHorBase = ReferenceTrsfHorFootInHorBase * T_RealStFootInHorStFoot;

//             // Corrected reference for the swing leg
//             DesTrsfSwingFootInCurBase = TrsfWorldIMUInCurBase * ReferenceTrsfSwingFootInHorBase;
//             //DesTrsfSwingFootInCurBase = ReferenceTrsfSwingFootInHorBase;

//             //
//             // DesTrsfLeftFootInHorBase  = DesTrsfRealStanceFootInHorBase;
//             // DesTrsfRightFootInHorBase = DesTrsfSwingFootInCurBase;

//             DesTrsfLeftFootInHorBase  = ReferenceTrsfHorFootInHorBase;
//             //DesTrsfLeftFootInHorBase  = TrsfCurBaseInWorldIMU * ReferenceTrsfHorFootInHorBase;
//             DesTrsfRightFootInHorBase = TrsfCurBaseInWorldIMU * ReferenceTrsfSwingFootInHorBase;


//         }
//         else   // the right foot is assumed to be the stance foot
//         {
//             StanceFootInCurBase = FwKRightFootInBase;
//             ReferenceTrsfHorFootInHorBase = GTj->TrsfRightLegBase;
//             ReferenceTrsfSwingFootInHorBase = GTj->TrsfLeftLegBase;

//             //

//             CurTrsfRealStanceFootInHorBase = TrsfCurBaseInWorldIMU * StanceFootInCurBase;
//             // extracting the rotation matrix
//             CurRotRealStFootInHorBase = CurTrsfRealStanceFootInHorBase.block(0, 0, 3, 3);

//             // orientation of the current stance foot wrt. the horizontal plane
//             Vector3d EulerXYZ_F_hB, OrientXY_F_hF;

//             EulerXYZ_F_hB = getEulerAnglesXYZ_FixedFrame(Rot_Base2BaseFoot * CurRotRealStFootInHorBase);

//             OrientXY_F_hF    = EulerXYZ_F_hB;
//             OrientXY_F_hF(2) = 0.0;

//             RotRealStFootInHorStFoot = getComposedOrientFixedFrame(OrientXY_F_hF);

//             // Desired Transformation of the real stance foot expressed in the Horizon base

//             // Trsf form real stance foot in horizontal stance foot frame
//             MatrixXd T_RealStFootInHorStFoot;

//             T_RealStFootInHorStFoot.resize(4,4);
//             T_RealStFootInHorStFoot.setZero(4,4);   T_RealStFootInHorStFoot(3,3) = 1.;
//             T_RealStFootInHorStFoot.block(0,0, 3,3) = RotRealStFootInHorStFoot;

//             cout << " EulerXYZ_F_hB is :\n"<< EulerXYZ_F_hB << endl;
//             cout << " T_RealStFootInHorStFoot is :\n"<< T_RealStFootInHorStFoot << endl;


//             DesTrsfRealStanceFootInHorBase = ReferenceTrsfHorFootInHorBase * T_RealStFootInHorStFoot;

//             // corrected reference for the swing
//             DesTrsfSwingFootInCurBase = TrsfWorldIMUInCurBase * ReferenceTrsfSwingFootInHorBase;
//             //DesTrsfSwingFootInCurBase = ReferenceTrsfSwingFootInHorBase;

//             //
//             // DesTrsfLeftFootInHorBase  = DesTrsfSwingFootInCurBase;
//             // DesTrsfRightFootInHorBase = DesTrsfRealStanceFootInHorBase;

//             DesTrsfLeftFootInHorBase  = TrsfCurBaseInWorldIMU * ReferenceTrsfSwingFootInHorBase;
//             //DesTrsfRightFootInHorBase = TrsfCurBaseInWorldIMU * ReferenceTrsfHorFootInHorBase;
//             DesTrsfRightFootInHorBase = ReferenceTrsfHorFootInHorBase;

//         }


// }

// void CpDesiredFeetTransformations::SetImuTransformation(yarp::sig::Vector IMU_RollPitchYaw)
// {
//     MatrixXd T_B_IMU;
//     T_B_IMU.resize(4,4);
//     T_B_IMU.setZero(4,4);   T_B_IMU(3,3) = 1.;

//     yarp::sig::Matrix R_B_IMU;

//     R_B_IMU = yarp::math::rpy2dcm(CTRL_DEG2RAD * IMU_RollPitchYaw);

//     // extract values from the rotation matrix of yarp::Matrix type and assign them to Eigen::MatrixXd
//     for (int row=0; row<3; row++)
//     {
//         for (int col=0; col<3; col++)
//         {
//             T_B_IMU(row,col) = R_B_IMU(row,col);
//         }
//     }


//     // Homogeneous transformation expressin the rotation of the base with respect the IMU_intertial frame
//     TrsfCurBaseInWorldIMU = T_B_IMU;


// }


// Vector3d CpDesiredFeetTransformations::getEulerAnglesXYZ_FixedFrame(Matrix3d CurRotRealStFootInHorBase)
// {
//     // this function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
//     // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX

//     VectorXd Angles(3);
//     double Psi_X, Theta_Y, Phi_Z;
//     const double PI = 3.14159265;

//     Psi_X   = atan2(CurRotRealStFootInHorBase(2,1),CurRotRealStFootInHorBase(2,2));
//     Theta_Y = atan2(-CurRotRealStFootInHorBase(2,0), abs(sqrt(pow(CurRotRealStFootInHorBase(0,0), 2.)+pow(CurRotRealStFootInHorBase(1,0), 2.))));
//     Phi_Z   = atan2(CurRotRealStFootInHorBase(1,0),CurRotRealStFootInHorBase(0,0));

//     if ((Theta_Y>PI/2.)||(Theta_Y<-PI/2.))
//     {
//         Psi_X   = atan2(-CurRotRealStFootInHorBase(2,1),-CurRotRealStFootInHorBase(2,2));
//         Theta_Y = atan2(-CurRotRealStFootInHorBase(2,0),-abs(sqrt(pow(CurRotRealStFootInHorBase(0,0), 2.)+pow(CurRotRealStFootInHorBase(1,0), 2.))));
//         Phi_Z   = atan2(-CurRotRealStFootInHorBase(1,0),-CurRotRealStFootInHorBase(0,0));
//     }

//     Angles(0) = Psi_X;
//     Angles(1) = Theta_Y;
//     Angles(2) = Phi_Z;

//     return Angles;

// }

// Matrix3d CpDesiredFeetTransformations::getComposedOrientFixedFrame(Vector3d OrientXY_F_hF)
// {
//     // this function computes the Euler composition rotation matrix for rotation angles given wrt. fixed frame
//     Matrix3d EulerXYZ;
//     EulerXYZ.setZero(3,3);

//     // OrientXY_F_hF(0): Psi_X
//     // OrientXY_F_hF(1): Theta_Y
//     // OrientXY_F_hF(2): Phi_Z

//     EulerXYZ(0,0) =  cos(OrientXY_F_hF(2))*cos(OrientXY_F_hF(1));
//     EulerXYZ(1,0) =  sin(OrientXY_F_hF(2))*cos(OrientXY_F_hF(1));
//     EulerXYZ(2,0) = -sin(OrientXY_F_hF(1));

//     EulerXYZ(0,1) = -sin(OrientXY_F_hF(2))*cos(OrientXY_F_hF(0))+cos(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*sin(OrientXY_F_hF(0));
//     EulerXYZ(1,1) =  cos(OrientXY_F_hF(2))*cos(OrientXY_F_hF(0))+sin(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*sin(OrientXY_F_hF(0));
//     EulerXYZ(2,1) =  cos(OrientXY_F_hF(1))*sin(OrientXY_F_hF(0));

//     EulerXYZ(0,2) =  sin(OrientXY_F_hF(2))*sin(OrientXY_F_hF(0))+cos(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*cos(OrientXY_F_hF(0));
//     EulerXYZ(1,2) = -cos(OrientXY_F_hF(2))*sin(OrientXY_F_hF(0))+sin(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*cos(OrientXY_F_hF(0));
//     EulerXYZ(2,2) =  cos(OrientXY_F_hF(1))*cos(OrientXY_F_hF(0));


//     return EulerXYZ;

// }

// ====================================================================================================================



// ====================================================================================================================
CpReactiveWalkingController::CpReactiveWalkingController(): DMod(0)
                                            , MpcModel(0)
                                            , SMx(0)
                                            , CoPref(0)
                                            , VeloRef(0)
                                            , QMx(0)
                                            , PVec(0)
                                            , CnstrFtStp(0)
                                            , CnstrZmp(0)
                                            , QPxySolver(0)
                                            , QPAngSolver(0)
                                            , FtTraj(0)
                                            , GaitTrsf(0)
                                            // , DesFtTrsf(0)
                                            {} 

CpReactiveWalkingController::~CpReactiveWalkingController()
{
    CpReactiveWalkingController::ReleaseCpBalWlkController();
}


void CpReactiveWalkingController::ReleaseCpBalWlkController()
{
  if (DMod) {
        delete DMod;
        DMod = 0;
    }
    if (MpcModel) {
        delete MpcModel;
        MpcModel = 0;
    }
    if (SMx) {
        delete SMx;
        SMx = 0;
    }
    if (CoPref) {
        delete CoPref;
        CoPref = 0;
    }
    if (VeloRef) {
        delete VeloRef;
        VeloRef = 0;
    }
    if (QMx) {
        delete QMx;
        QMx = 0;
    }
    if (PVec) {
        delete PVec;
        PVec = 0;
    }
    if (CnstrFtStp) {
        delete CnstrFtStp;
        CnstrFtStp = 0;
    }
    if (CnstrZmp) {
        delete CnstrZmp;
        CnstrZmp = 0;
    }
    if (QPxySolver) {
        delete QPxySolver;
        QPxySolver = 0;
    }
    if (QPAngSolver) {
        delete QPAngSolver;
        QPAngSolver = 0;
    }
    if (FtTraj) {
        delete FtTraj;
        FtTraj = 0;
    }
    if (GaitTrsf) {
        delete GaitTrsf;
        GaitTrsf = 0;
    }
    // if (DesFtTrsf) {
    //     delete DesFtTrsf;
    //     DesFtTrsf = 0;
    // }

}


void CpReactiveWalkingController::InitializeCpBalWlkController(InitBalWlkParameters *Parameters)
{
  // initialization of the counter
    SwitchCounter = 0;

    CtrlCounter    = 0;
    WaitForContact = false;
    // initialisation of optimal Footsteps solution
    RelCoPXYR.resize(11);
    RelCoPXYR.setZero(11);

    RelCoPXYR_n1.resize(3);
    RelCoPXYR_n1.setZero(3);

    // Optimal control signal at t-1
    OptimSolXY_n1.resize(2);
    OptimSolXY_n1.setZero(2);

    // Optimal control signal at t
    RelCoPXY.resize(6);
    RelCoPXY.setZero(6);

    OrienCoP.resize(3);
    OrienCoP.setZero(3);

    // LIPM object
    DMod = new Discrete_CP_LIP_Model(Parameters->SamplingTime, Parameters->CoMHeight, Parameters->CpModelOption);
    DMod->Create_CP_LIP_Model(Parameters->SamplingTime, Parameters->CoMHeight);

    // MPC Object
    MpcModel = new MpcBased_CP_Model;
    MpcModel->CreateSampMpc_CP_Model( DMod, Parameters->SamplesVector, Parameters->SamplesPerStep, Parameters->DurationSteps, Parameters->CpModelOption);
    MpcModel->getEmx();
    MpcModel->prefactorization();

//    cout << "Pps is \n"<< MpcModel->Pps << endl;
//    cout << "Pvs is \n"<< MpcModel->Pvs << endl;
//    cout << "PEs is \n"<< MpcModel->PEs << endl;

    // Cyclic selection matrices
    SMx = new CP_SelectionMatrices ( Parameters->SamplesVector, Parameters->SamplingTime, Parameters->SamplesPerStep, Parameters->DurationSteps, Parameters->CoMHeight);

    // Center of Pressure (CoP) refernce Trajectory
    CoPref = new CpStanceFootPose (Parameters->InitCoPrefPose);

    // Compute Reference velocity in inertial frame
    VeloRef = new VelocitiesSetPoints;
    VeloRef->WorldVelocitiesSetPts(SMx, Parameters->InitialVelocity);

    // Hessian Matrix
    QMx = new CpQMatrix(MpcModel,Parameters->gains, Parameters->CpModelOption);

    MatrixXd MQKxy, MQang;
    MQKxy = QMx->getQMxy(MpcModel, SMx);
    MQang = QMx->getQMang(MpcModel, SMx);


    // Gradient vector
    PVec = new CpPVectorQP(MpcModel, Parameters->gains, Parameters->CpModelOption);

    VectorXd VPKxy, VPang;
    VPKxy = PVec->getVeckxy( MpcModel, DMod, CoPref, SMx, VeloRef, OptimSolXY_n1);
    VPang = PVec->getVeckAng(MpcModel, DMod, CoPref, SMx, VeloRef);

    // constraints of the footsteps placement
    CnstrFtStp = new CpConstraintsFootsteps(CoPref, Parameters->StanceIndicator, Parameters->CtrlHorizon, Parameters->SupportConstraints);
    CnstrFtStp->UpdateFtstpConstraints(CoPref, Parameters->StanceIndicator);

    // constarints on the center of Pressure (ZmP)
    CnstrZmp = new CpConstraintsOutputZmp(MpcModel, SMx, DMod, CoPref, Parameters->CpModelOption);
    CnstrZmp->FindReducedZmpConstraints12(MpcModel, SMx, DMod, CoPref);

    // matrix of contraints on the CoP and the ZMP

    if (Parameters->FootStepsCnstrOnly)
    {
        // constraints matrix
        MconsXY.resize((CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.setZero((CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols()) = CnstrFtStp->Mxy;
        // constrainst vector
        BconsXY = CnstrFtStp->Bxy;
    }
    else
    {
        // constraints matrix
        MconsXY.resize((CnstrZmp->MuAct).rows()+(CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.setZero((CnstrZmp->MuAct).rows()+(CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols()) = CnstrFtStp->Mxy;
        MconsXY.block((CnstrFtStp->Mxy).rows(), 0, (CnstrZmp->MuAct).rows(), (CnstrZmp->MuAct).cols()) = CnstrZmp->MuAct;
        // constraints vector
        BconsXY.resize((CnstrFtStp->Bxy).rows() + (CnstrZmp->BuAct).rows());
        BconsXY.segment(0, (CnstrFtStp->Bxy).rows()) = CnstrFtStp->Bxy;
        BconsXY.segment((CnstrFtStp->Bxy).rows(), (CnstrZmp->BuAct).rows()) = CnstrZmp->BuAct;
    }


    //int_t nWSRAng = 200;
    Options QPAngOptionToSet;
    QPAngOptionToSet.setToMPC();
    QPAngOptionToSet.printLevel = PL_LOW;

    // Initializing the solver for the translations
    QPxySolver =  new CP_QPSolver_OASES;
    QPxySolver->InitialiseQPSol(MQKxy, VPKxy, MconsXY, BconsXY);
    QPxySolver->setSolverOptions(QPAngOptionToSet);
    QPxySolver->CP_QPSolution_Oases_XY(MQKxy, VPKxy, MconsXY, BconsXY);

    // Initializing the solver for the orientation
    QPAngSolver = new CP_QPSolver_OASES;
    QPAngSolver->InitialiseQPSol(MQang, VPang, CnstrFtStp->Mang, CnstrFtStp->Bang);
    QPAngSolver->setSolverOptions(QPAngOptionToSet);
    QPAngSolver->CP_QPSolution_Oases_Ang(MQang, VPang, CnstrFtStp->Mang, CnstrFtStp->Bang);

    // Feet reference trajectories generator

    FtTraj = new CpFootTrajectories(Parameters->StanceIndicator, RelCoPXYR, CoPref, SMx, Parameters->maxFootHeight);

    // Reference Homogeneous transformations for the legs
    GaitTrsf =  new CpGaitTransformations(DMod, Parameters->StanceIndicator, CoPref, FtTraj, Parameters->Translation_B_CoM);

    // Desired Feet transformation to be send to the inverse kinematics module of the robot
    // DesFtTrsf = new CpDesiredFeetTransformations();   // InertialCompensator

}

void CpReactiveWalkingController::UpdateCpBalWlkController(InitBalWlkParameters *Parameters, VectorXd RelativeVelo, int CycleCounter)
{

    //VectorXd RelativeVelo(3);

    // RelativeVelo(0) = -0.00;  // TO DO
    // RelativeVelo(1) = 0.00;
    // RelativeVelo(2) = 0.00;

    //
    DTds = 2*Parameters->SamplingTime;
    Tgbs = (int)(round((Parameters->DurationSteps[0]-DTds)/Parameters->SamplingTime));
    Tgas = (int)(round(DTds/Parameters->SamplingTime));

    
    // **********************************************************
    // Selection of initial Stance foot
    // *********************************
    if (CtrlCounter == 0)
    {
        if (RelativeVelo(1) >= 0)
        {
          // Right support foot
            // Stance Foot (CoP) reference pose
            Parameters->InitCoPrefPose(1) = -0.5* Parameters->SupportConstraints(2); //-0.055; //0.0681
            Parameters->InitCoPrefPose(4) = -0.5* Parameters->SupportConstraints(2); //-0.055;
            CoPref->ReinitializeFootPose(Parameters->InitCoPrefPose);
            DMod->StatesY(1)=CoPref->yi_0(0);
            if(Parameters->RobotName == "icub"){DMod->StatesY(1)= 0.5 *CoPref->yi_0(0);}
            RelCoPXYR_n1(1) = -Parameters->SupportConstraints(2); //-0.11;
            SwitchCounter = 1;
        }
        else
        {
            // Left support foot
            // Stance Foot (CoP) reference pose
            Parameters->InitCoPrefPose(1) = 0.5* Parameters->SupportConstraints(2); //0.055;
            Parameters->InitCoPrefPose(4) = 0.5* Parameters->SupportConstraints(2); //0.055;
            CoPref->ReinitializeFootPose(Parameters->InitCoPrefPose);
            DMod->StatesY(1)= CoPref->yi_0(0);
            RelCoPXYR_n1(1) = Parameters->SupportConstraints(2); //0.11;
            SwitchCounter = 0;
        }
        // update the cyclic Selection matrices
        SMx->CP_UpdateSelectionMx(CoPref->CoPRefR, QPAngSolver->OrienCoP, CtrlCounter);
    }
    else
    {
        // update the cyclic Selection matrices
        SMx->CP_UpdateSelectionMx(CoPref->CoPRefR ,QPAngSolver->OrienCoP, CtrlCounter);

        // support foot indicator index
        if (SMx->IndexSFt == 0) { SwitchCounter += 1;}
    }

        // -------------------------------------
        if (fmod(SwitchCounter,2) == 0) {
            Parameters->StanceIndicator[0] = 1; // Left  Uk_0
            Parameters->StanceIndicator[1] = 0; // Right Uc_k_0
        }
        else {
            Parameters->StanceIndicator[0] = 0;
            Parameters->StanceIndicator[1] = 1;
        }

    // Compute the absolute velocty to be applied
    VeloRef->WorldVelocitiesSetPts(SMx, RelativeVelo);

    // Update the Pose the robot with the QP solution
    CoPref->UpdateStanceFootPose(SMx, RelCoPXYR, RelCoPXYR_n1);

    // Update of the Hessian Matrix
    MatrixXd MQKxy, MQang;
    VectorXd VPKxy, VPang;

    MQKxy = QMx->getQMxy(MpcModel, SMx);
    MQang = QMx->getQMang(MpcModel, SMx);

    // Update of the Gradient vector
    OptimSolXY_n1(0) = QPxySolver->uX;
    OptimSolXY_n1(1) = QPxySolver->uY;


//          cout<< " Agular position X is : \n" << CoPref->CoPRefR << endl;

    VPKxy = PVec->getVeckxy(MpcModel, DMod, CoPref, SMx, VeloRef, OptimSolXY_n1);
    VPang = PVec->getVeckAng(MpcModel, DMod, CoPref, SMx, VeloRef);

    // Update the Constraints on the Footsteps
    CnstrFtStp->UpdateFtstpConstraints(CoPref, Parameters->StanceIndicator);

    // Determine the constraints on the Center of Pressure
    // adapting the constraints to the DSP

    if ((SMx->IndexSFt >=Tgbs)||(SMx->IndexSFt <=Tgas))
    {
        if (Parameters->StanceIndicator[1] == 1)
        {
            Spolygon_limits.resize(4);
            Spolygon_limits(0) = 0.150; Spolygon_limits(1) = 0.20; //0.02;
            Spolygon_limits(2) = 0.150; Spolygon_limits(3) = 0.03; //0.02;

            CnstrZmp->SetSupportEdgesLimits(Spolygon_limits);
        }
        else
        {
            Spolygon_limits.resize(4);
            Spolygon_limits(0) = 0.150; Spolygon_limits(1) = 0.03; //0.02;
            Spolygon_limits(2) = 0.150; Spolygon_limits(3) = 0.20; //0.02;

            CnstrZmp->SetSupportEdgesLimits(Spolygon_limits);
        }
    }
    else
    {
        Spolygon_limits.resize(4);
        Spolygon_limits(0) = 0.070; Spolygon_limits(1) = 0.03; //0.02;
        Spolygon_limits(2) = 0.070; Spolygon_limits(3) = 0.03; //0.02;

        CnstrZmp->SetSupportEdgesLimits(Spolygon_limits);
    }

    CnstrZmp->FindReducedZmpConstraints12(MpcModel,SMx, DMod, CoPref);

    // matrix of contraints on the CoP and the ZMP
    if (Parameters->FootStepsCnstrOnly)
    {
        // constraints matrix
        MconsXY.resize((CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.setZero((CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols()) = CnstrFtStp->Mxy;
        // constrainst vector
        BconsXY = CnstrFtStp->Bxy;
    }
    else
    {
        // constraints matrix
        MconsXY.resize((CnstrZmp->MuAct).rows()+(CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.setZero((CnstrZmp->MuAct).rows()+(CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp->Mxy).rows(), (CnstrFtStp->Mxy).cols()) = CnstrFtStp->Mxy;
        MconsXY.block((CnstrFtStp->Mxy).rows(), 0, (CnstrZmp->MuAct).rows(), (CnstrZmp->MuAct).cols()) = CnstrZmp->MuAct;
        // constraints vector
        BconsXY.resize((CnstrFtStp->Bxy).rows() + (CnstrZmp->BuAct).rows());
        BconsXY.segment(0, (CnstrFtStp->Bxy).rows()) = CnstrFtStp->Bxy;
        BconsXY.segment((CnstrFtStp->Bxy).rows(), (CnstrZmp->BuAct).rows()) = CnstrZmp->BuAct;
    }

    // Get the optimal solution
    nWSR = 450;  //200

    //
    QPxySolver->setnbWorkingSetRecalculation(nWSR);
    QPxySolver->CP_QPSolution_Oases_XY(MQKxy, VPKxy, MconsXY, BconsXY);

    QPAngSolver->setnbWorkingSetRecalculation(nWSR);
    QPAngSolver->CP_QPSolution_Oases_Ang(MQang, VPang, CnstrFtStp->Mang, CnstrFtStp->Bang);

    // update of the states
    DMod->UpdateStates_CP_LIP_M(QPxySolver->uX, QPxySolver->uY, QPAngSolver->uR);

    // FOOT TRAJECTORIES
    FtTraj->ComputeFootTrajectories(Parameters->StanceIndicator, RelCoPXYR, CoPref, SMx);

    //cout << "RelCoPXYR is = "<< RelCoPXYR << endl;

    // Legs Transformations
    // Update the position of the base wrt to CoM.
    GaitTrsf->SetTranslationBaseCoM(Parameters->Translation_B_CoM);
    // Compute the gait transformations
    GaitTrsf->ComputeGaitTransforms(DMod, Parameters->StanceIndicator, CoPref, FtTraj);

    //Update the Pose the robot with the QP solution
    RelCoPXYR.segment(0, RelCoPXY.rows()) = QPxySolver->RelCoPXY;
    RelCoPXYR.segment(6, 2)               = QPxySolver->RelEoSXY;
    RelCoPXYR.segment(8, OrienCoP.rows()) = QPAngSolver->OrienCoP;

    RelCoPXYR_n1(0) = QPxySolver->RelCoPXY_n1(0);
    RelCoPXYR_n1(1) = QPxySolver->RelCoPXY_n1(1);
    RelCoPXYR_n1(2) = QPAngSolver->OrienCoP_n1(0);

    VectorXd zx_ref = SMx->CP_GlobalCurrentSelX * CoPref->CoPRefX + SMx->CP_GlobalFutureSelX * QPxySolver->RelCoPXY;
    VectorXd zy_ref = SMx->CP_GlobalCurrentSelY * CoPref->CoPRefY + SMx->CP_GlobalFutureSelY * QPxySolver->RelCoPXY;


    CoPref->xCP_ref = (SMx->FeE_t * QPxySolver->RelEoSXY(0) + SMx->Fep_t * zx_ref)(0);

    CoPref->yCP_ref = (SMx->FeE_t * QPxySolver->RelEoSXY(1) + SMx->Fep_t * zy_ref)(0);



    if(WaitForContact)
    {
        CtrlCounter = CtrlCounter;
    }
    else
    {
        CtrlCounter ++;       
    }

}


// ===============================================================================================

// ===============================================================================================

SyncRobotModelStanceFoot::SyncRobotModelStanceFoot() {}

SyncRobotModelStanceFoot::~SyncRobotModelStanceFoot() {}

void SyncRobotModelStanceFoot::InitStanceSync(double F_Threshld, int T_Threshld)
{
    // default values
        minForceThrshld = 10.0;
        minTimeThrshld = 5; 
        // user defined values
        minForceThrshld = F_Threshld;
      minTimeThrshld  = T_Threshld; 
}

void SyncRobotModelStanceFoot::SyncStancePhase(               int CycleCounter,
                                                InitBalWlkParameters *Parameters,
                                                CpReactiveWalkingController *CpBalWlkController,
                                                         VectorXd l_foot_FT_vector,
                                                         VectorXd r_foot_FT_vector)
{
  if (fabs(l_foot_FT_vector(2))-fabs(r_foot_FT_vector(2)) > minForceThrshld)
  {
    stance_left  = 1;
    stance_right = 0;
  }
  else if (fabs(r_foot_FT_vector(2))-fabs(l_foot_FT_vector(2)) > minForceThrshld)
  {
    stance_left  = 0;
    stance_right = 1;
  }

  if (CycleCounter <= minTimeThrshld)
  {
    stance_left  = Parameters->StanceIndicator[0];
    stance_right = Parameters->StanceIndicator[1];
  }

  // wait for the supposed support foot to touch the ground
  // check the stance as per the model and the actual stance foot measured 
  // by the torque/ force sensors

  if (Parameters->SwitchingSync)
  {
    if (Parameters->StanceIndicator[0] == 1 && stance_left !=1)
    {
      // wait
      CpBalWlkController->WaitForContact = true;
    }
    else if (Parameters->StanceIndicator[1] == 1  && stance_right !=1)
    {
      // wait
      CpBalWlkController->WaitForContact = true;
    }
    else
    {
      CpBalWlkController->WaitForContact = false;
    }
  }

}

// ====================================================================================================================



// ====================================================================================================================