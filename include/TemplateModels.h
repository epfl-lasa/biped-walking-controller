
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

/* This class is made of two subclasses: one that encodes the simplified model of 
   a biped humanoid robot (here a 3D linear inverted pendulum) and another class that
   encodes the prediction model associated to the chosen template model 
*/



#ifndef TemplateModels_H
#define TemplateModels_H 

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;



// ===============================================================================================
/*
 * Discrete_CP_LIP_Model_H : This class encodes the discrete template model
 * used to model the dynamics of biped robot standing on one foot.
 *
 * The models are based on the 3D LIPM with finite sized foot.
 * 
*/
// ===============================================================================================


// class CP_LIP_Model
class Discrete_CP_LIP_Model
{


        double g;   // Gravity acceleration

    public :

        double Ts;          // State samplimg time
        double zc;          // Height of the robot CoM

        double W;
        double a1;
        double b1;

        MatrixXd MxA;       // State Transition matrix
        VectorXd VeB;    // State Control vector
        RowVectorXd VeC;    // Stace Measurement Vector for ZMP

        MatrixXd MxA_R;       // State Transition matrix
        VectorXd VeB_R;    // State Control vector
        RowVectorXd VeC_R;   // Stace Measurement Vector for stance foot orientation
        RowVectorXd VeCz;    // Stace Measurement Vector for ZMP

        VectorXd StatesX;
        VectorXd StatesY;
        VectorXd StatesR;

        VectorXd StatesDistX;
        VectorXd StatesDistY;
        VectorXd StatesDistR;

        double OutputX;
        double OutputY;
        double OutputR;

        double CtrlUX;      // control input (X CoM jerk)
        double CtrlUY;      // control input (Y CoM jerk)
        double CtrlUR;      // control input (Theta jerk)

        double ZMP_X;
        double ZMP_Y;


        double Mass;
        double AlpFrc;
        VectorXd B_Dist;

        int CpModelOption; //Type of LIPM model used for MPC


        // Constructor of the class Discrete_CP_LIP_Model
        Discrete_CP_LIP_Model(double x, double y, int CpOption);

        ~Discrete_CP_LIP_Model();
        //DiscreteLIPModel(double SpTime, double CoMHeight);
        void Create_CP_LIP_Model(double SpTime, double CoMHeight);

        // Update of the state by LMPC optimal solution

       void UpdateStates_CP_LIP_M(double uX, double uY, double uR);

        // set the height of the CoM
       void SetCoMHeight(double CoMHeight);

       // Get the height of the CoM
};

// ===================================================================================================
/*
 * MpcBased_CP_Model : This class creates the prediction model associated with 
 * the chosen template model.
 *
*/
// ===================================================================================================

class MpcBased_CP_Model {

    double g; // gravity acceleration
    VectorXd SamplesVector; // vector of sampling instants over horizon Ny

    public :

    int Ny        ; // Receding Horizon
    int Nu        ; // Control Horizon

    double Zc     ; // CoM Height
    double Tp     ; // Sampling Time

    MatrixXd Pps    ; // sub-state transition matrix of CoM position over Ny*T
    MatrixXd Pvs    ; // sub-state transition matrix of CoM velocity over Ny*T
    MatrixXd Pes    ; // sub-state transition matrix of CoM velocity over Ny*T
    MatrixXd PEs    ; // sub-state transition matrix of capture point position over Ny*T
    MatrixXd Pzs    ; // sub-state transition matrix of ZMP position over Ny*T

    MatrixXd PpsR    ; // sub-state transition matrix of CoM position over Ny*T
    MatrixXd PvsR    ; // sub-state transition matrix of CoM velocity over Ny*T
    MatrixXd PzsR   ; // sub-state transition matrix of stance foot orientation over Ny*T

    MatrixXd Ppu    ; // sub-state control matrix of CoM position over Ny*T
    MatrixXd Pvu    ; // sub-state control matrix of CoM velocity over Ny*T
    MatrixXd Peu    ; // sub-state control matrix of CoM acceleration over Ny*T
    MatrixXd PEu    ; // sub-state control matrix of capture point position over Ny*T
    MatrixXd Pzu    ; // sub-state control matrix of ZMP position over Ny*T

    MatrixXd PpuR    ; // sub-state control matrix of CoM position over Ny*T
    MatrixXd PvuR    ; // sub-state control matrix of CoM velocity over Ny*T
    MatrixXd PzuR   ; // sub-state control matrix of stance foot orientation over Ny*T

    MatrixXd TheP;
    VectorXd e1;
    MatrixXd Emx; // matrix used to compute the CoM average velocity from position

    // for the Dynamic Filter
    MatrixXd PEsDf;  
    MatrixXd PEuDf;
    MatrixXd ThePDf;
    VectorXd e1_Df;

    MatrixXd PEuDf_T_PEuDf;
    MatrixXd ThePDf_T_ThePDf;
    MatrixXd PEuDf_T_PEsDf;
    VectorXd PEuDf_T_ones_ns1;
    VectorXd ThePDf_T_e1_Df;

    // Prefactorisation for computation purposes
    MatrixXd PpuR_T_Emx_T;
    VectorXd TheP_T_e1    ;
    MatrixXd Ppu_T_Emx_T  ;
    MatrixXd Emx_Pps      ;

    double DurationSteps[5]; // array of duration of the 4 steps within Horizon Ny
    int SamplesPerStep[4]; // array of number of samples for each steps

    int CpModelOption; //Type of LIPM model used for MPC


    void CreateContMpc_CP_Model(double, //SamplingTime,
                                int, //RecHorizon
                                int, //CtrlHorizon
                                double, //CoMHeight
                                int );  // LIPM model option

    void CreateSampMpc_CP_Model(Discrete_CP_LIP_Model *DMod, // Discrete LIPM Object
                                VectorXd SampVec, //Sample vector
                                int nbSampStp[], // array of nb of samples per steps
                                double DurStep[], // array of steps durations
                                int CpOption  );

    void prefactorization();

    MatrixXd getEmx(); // matrix used to compute the CoM average velocity from position

    MatrixXd A(double Ti, double zc);

    VectorXd B(double Ti, double zc);

    void CpDynamicFilter(Discrete_CP_LIP_Model *DMod, // Discrete LIPM Object
                                    VectorXd SampVec, //Sample vector
                                    int nbSampStp[], // array of nb of samples per steps
                                    double DurStep[], // array of steps durations
                                    int CpOption);

};




   // ====================================================================================
/*
 * CP_SelectionMatrices : This class computes the cyclic selection vector and
 * matrices associating each sampling instant to footstep poses and reference
 * capture-point value.
 *
*/
// ====================================================================================

class CP_SelectionMatrices
{

  // Vector and Matrices

  // sub-matrices of Fep_t (see below)
  MatrixXd Fep12;
  MatrixXd Fep23;
  MatrixXd Fep34;
  // sub-vector of FeE_t (see below)
  VectorXd FeE12;
  VectorXd FeE23;
  VectorXd FeE34;

  VectorXd Uc_12;
  VectorXd Uc_23;
  VectorXd Uc_34;

  MatrixXd  Uf_12;
  MatrixXd  Uf_23;
  MatrixXd  Uf_34;

  VectorXd U0;
  VectorXd Uf1;

  int  na;    // counter of ny cycles within a12
  int  na3;   // counter of ny cycles within a23
  int  time_1;


  public :

  VectorXd SamplesVector;                 // vector of samples over RecH
  int SpS[4];                             // array of nb of samples per steps
  double DrS[5];                          // array of steps durations
                                          // [0] - [3]: single support duration of steps 1,2,3 and 4
                                          // [5] duration of double support phase


  double Tp;                              // sampling time
  double zc;                              // Height of CoM
  double W;                               // Natural frequency of the LIPM

  // selection matrix of the stance foot defined wrt. the robot
  VectorXd LocalCurrentSel;               // current
  MatrixXd LocalFutureSel;                // future (predicted steps)

  VectorXd GlobalCurrentSelX;             // current
  MatrixXd GlobalFutureSelX;              // future (predicted steps)

  VectorXd GlobalCurrentSelY;             // current
  MatrixXd GlobalFutureSelY;              // future (predicted steps)


  VectorXd CP_LocalCurrentSel;            // constant selection vector of current footstep
  MatrixXd CP_LocalFutureSel;             // constant selection matrix of future footsteps

  // selection matrix of the stance foot defined wrt. the world frame
  VectorXd CP_GlobalCurrentSelX;          // current
  MatrixXd CP_GlobalFutureSelX;           // future (predicted steps)

  VectorXd CP_GlobalCurrentSelY;          // current
  MatrixXd CP_GlobalFutureSelY;           // future (predicted steps)

  // Selection matrix and Vector associating footsteps to reference Capture-point
  MatrixXd Fep_t;
  VectorXd FeE_t;

  // X
  MatrixXd GlobalCurrent_Fep_t_X;         // current footstep
  MatrixXd GlobalFuture_Fep_t_X;          // future footsteps

  MatrixXd GlobalFuture_FeE_t_X;          // end of step capture-point X
  // Y
  MatrixXd GlobalCurrent_Fep_t_Y;         // current footstep
  MatrixXd GlobalFuture_Fep_t_Y;          // future footsteps

  MatrixXd GlobalFuture_FeE_t_Y;          // end of steps capture-point Y

  // selector of stance foot
  int LeftStanceFoot;
  int RightStanceFoot;

  int RecH; // receding horizon

  MatrixXd RotMx;                        // X rotation matrix of relative footsteps
  MatrixXd RotMy;                        // Y rotation matrix of relative footsteps
  int IndexSFt;                          // cyclic variable indication change in Sft
  VectorXd Theta_k1;                     // stance foot orientation over Ny

  RowVectorXd RotX3;
  RowVectorXd RotX33;
  RowVectorXd RotY3;
  RowVectorXd RotY33;

  CP_SelectionMatrices(VectorXd SampVec,
                         double SamplingTime,
                            int SamplesPerStep[],
                         double DurationSteps[],
                         double CoMHeight);

  void CP_UpdateSelectionMx(  double xk_fc_ang,
                            VectorXd OrienCoP,
                                 int time_index);

};


// // ====================================================================================



// // ====================================================================================
class CpStanceFootPose
{


       public :

           double CoPRefX ; // X CoP positon with respect to the world frame
           double CoPRefY ; // Y CoP positon with respect to the world frame
           double CoPRefR ; // stance foot orientation with respect to the
                            // world frame

           double xCP_ref;  // reference capture point
           double yCP_ref;  // reference capture point

           VectorXd xi_0; // X predicted footsteps positions with respect
                                // to the world frame
           VectorXd yi_0; // [-0.05;0;0];  % Y predicted footsteps positions with respect
                                // to the world frame

           VectorXd xk_fc_xy_n1; // = -xk_f_xy(1:2,1);

           double xk_fc_ang_n1; // = xk_fc_ang;


           CpStanceFootPose(VectorXd IniCoPrefPose);

           void ReinitializeFootPose(VectorXd IniCoPrefPose);


           void UpdateStanceFootPose(CP_SelectionMatrices *SMx,
                                     VectorXd RelCoPXYR,
                                     VectorXd RelCoPXYR_n1);

};

// // ====================================================================================



// // ====================================================================================


class VelocitiesSetPoints
{




       public :

           VectorXd VxAbs;   //   X velocity w.r.t absolute (world) frame
           VectorXd VyAbs;   //  Y velocity w.r.t absolute (world) frame
           VectorXd WzAbs;   //  Omega Z velocity w.r.t absolute (world) frame

           void WorldVelocitiesSetPts(CP_SelectionMatrices *SMx, // Selection Matrices
                                              VectorXd R_Velo); // Relative velocity vector (v_x, v_y, w_z)

};




#endif // TemplateModels_H