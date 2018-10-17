
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

// MPCOptimizer
// 	1. class CP_QPSolver_OASES 		: the QP solver
// 	2. class CpQMatrix				: the Hessiam Matrix
// 	3. class CpPVectorQP			: the Gradient Vector
// 	4. class CpConstraintsFootsteps	: the Footstep Constraints
// 	5. class CpConstraintsOutputZmp : the ZMP Constraints

#ifndef MPCOptimizer_H 
#define MPCOptimizer_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/LU>

#include <qpOASES.hpp>

#include "TemplateModels.h"
// #include "PatternsGenerator.h"



USING_NAMESPACE_QPOASES

using namespace std;
using namespace Eigen;

// ===================================================================================
/*
 * CP_QPSolver_OASES : This class encodes a QP solver based on 
 * qpOASES library 
 *
*/
// ===================================================================================


class CP_QPSolver_OASES
{
   // members
       // ux
       VectorXd uXnu;  // X vector of control input (jerk) over the horizon nu

       // uy
       VectorXd uYnu;  // Y vector of control input (jerk) over the horizon nu

       // u_theta
       VectorXd uRnu;  // vector of angular control input over nu


   public :

       double uX       ;  // X current control action (jerk)
       double uY       ;  // Y current control input (jerk)
       double uR       ;  // current angular control input

       double uX_n1       ;  // X current control action (jerk)
       double uY_n1       ;  // Y current control input (jerk)
       double uR_n1       ;  // current angular control input

       // xk_f_x
       VectorXd RelCoPXY;    // Vector of predicted relative reference CoP

       VectorXd RelEoSXY;    // Vector of predicted relative End of Step of the Capture Point

       VectorXd OrienCoP;    // vector of predicted relative reference orientation

       VectorXd RelCoPXY_n1; // Vector of predicted relative reference CoP at Ts(n-1)

       VectorXd OrienCoP_n1; // vector of predicted relative reference orientation at Ts(n-1)

       SQProblem OasesSolver;

       //real_t xOpt[];
       //int_t nbWorkSetRecal;
       int nbWorkSetRecal;

       // Solving the QP problem

       CP_QPSolver_OASES();
       ~CP_QPSolver_OASES();

       void InitialiseQPSol(MatrixXd H,
                            VectorXd f,
                            MatrixXd A_cons,
                            VectorXd b);

       VectorXd qpOasesSolver (MatrixXd H,
                               VectorXd f,
                               MatrixXd A_cons,
                               VectorXd b);

       void setnbWorkingSetRecalculation(int nWSR);

       void CP_QPSolution_Oases(MatrixXd Q,         // Hessian matrix
                                VectorXd p_k,       // gradient vector
                                MatrixXd M,         // Matrix of constraints
                                VectorXd gam);      // vector of constraints

       void CP_QPSolution_Oases_Ang(MatrixXd Q,     // Hessian matrix
                                    VectorXd p_k,   // gradient vector
                                    MatrixXd M,     // Matrix of constraints
                                    VectorXd gam);  // vector of constraints

       void CP_QPSolution_Oases_XY(MatrixXd Q,      // Hessian matrix
                                   VectorXd p_k,    // gradient vector
                                   MatrixXd M,      // Matrix of constraints
                                   VectorXd gam);   // vector of constraints

       void CP_QPSolution_Oases_1T(MatrixXd Q,      // Hessian matrix
                                   VectorXd p_k,    // gradient vector
                                   MatrixXd M,      // Matrix of constraints
                                   VectorXd gam);   // vector of constraints

       void setSolverOptions(Options optionToSet);


};
   
// ===================================================================================
/*
 * CpQMatrix : This class encodes the Hessian matrix related to the MPC QP problem
*/
// ===================================================================================


class CpQMatrix 
{


           public :

           double GainsVec[8]; // Vector of QP gains

           MatrixXd q11trl_x; // Constant sub-matrix of Q translation
           MatrixXd q11trl_y; // Constant sub-matrix of Q translation
           MatrixXd q11Ang; // Constant sub-matrix of Q orientation

           MatrixXd Qang;   // Variable sub-matrix of Q orientation
           MatrixXd MxQk;

           MatrixXd MQx;   // X matrix as independent component
           MatrixXd MQy;   // Y matrix as independent component
           MatrixXd MQxy;   // Y matrix as independent component

           int CpModelOption; // Type of LIPM model

           CpQMatrix(MpcBased_CP_Model *MpcModel,
                       double GainsArray[], int CpOption);

           MatrixXd getQMx(MpcBased_CP_Model *MpcModel,
                                         CP_SelectionMatrices *SMx);
           MatrixXd getQMxy(MpcBased_CP_Model *MpcModel,
                                         CP_SelectionMatrices *SMx);
           MatrixXd getQMang(MpcBased_CP_Model *MpcModel,
                                         CP_SelectionMatrices *SMx);


           void QmxElements(MpcBased_CP_Model *MpcModel,
                                    CP_SelectionMatrices *SMx);

   };


// ===================================================================================
/*
 * CpPVectorQP : This class encodes the Gradient Vector related to the MPC QP problem
*/
// ===================================================================================

class CpPVectorQP
   {


       public :

       double GainsVec[8]; //

       MatrixXd P1trl; // Constant part of vector P translation
       MatrixXd P1Ang; // Constant part of vector P orientation

       VectorXd Pkx1;  // Variable part of vector P translation
       VectorXd Pkx2;  //
       VectorXd Pkx3;  //
       VectorXd Pky1;  //
       VectorXd Pky2;  //
       VectorXd Pky3;  //

       VectorXd PkAng1; // Variable part of vector P orientation
       VectorXd PkAng2; // Variable part of vector P orientation

       VectorXd VecPxy;  // Vector related to x and y
       VectorXd VecPx;  // Vector related to x as independent variable
       VectorXd VecPy;  // Vector related to y as independent variable

       VectorXd VecPAng;  // Vector related to x as independent variable


       VectorXd VecPk; //

       int CpModelOption;  // Type of LIPM Model


       CpPVectorQP(MpcBased_CP_Model *MpcModel,
                         double GainsArray[], int CpOption);


       void PVecElements(MpcBased_CP_Model *MpcModel, // LMPC model of the LIPM
                         Discrete_CP_LIP_Model *St, // CoM State variables
                         CpStanceFootPose *CoPRef,  // Stance feet pose
                         CP_SelectionMatrices *SMx,    // Selection matrices
                         VelocitiesSetPoints *VeloRef, // reference velocities wrt. the world frame
                         VectorXd OptimSolXY_n1);

       VectorXd getVeckx(MpcBased_CP_Model *MpcModel,    // LMPC model of the LIPM
                         Discrete_CP_LIP_Model *St,      // CoM State variables
                         CpStanceFootPose *CoPRef,  // Stance feet pose
                         CP_SelectionMatrices *SMx,    // Selection matrices
                         VelocitiesSetPoints *VeloRef, // reference velocities wrt. the world frame
                         VectorXd OptimSolXY_n1);


       VectorXd getVeckxy(MpcBased_CP_Model *MpcModel,    // LMPC model of the LIPM
                         Discrete_CP_LIP_Model *St,      // CoM State variables
                         CpStanceFootPose *CoPRef,  // Stance feet pose
                         CP_SelectionMatrices *SMx,    // Selection matrices
                         VelocitiesSetPoints *VeloRef, // reference velocities wrt. the world frame
                         VectorXd OptimSolXY_n1);

       VectorXd getVeckAng(MpcBased_CP_Model *MpcModel,    // LMPC model of the LIPM
                            Discrete_CP_LIP_Model *St,      // CoM State variables
                            CpStanceFootPose *CoPRef,  // Stance feet pose
                            CP_SelectionMatrices *SMx,    // Selection matrices
                            VelocitiesSetPoints *VeloRef); // reference velocities wrt. the world frame





   };
   
// ===================================================================================
/*
 * CpConstraintsFootsteps : This class encodes the constraints iimposed on the
 * the footstep positions and orientation.
 * More specifically, its imposes;
 *    - Maximum forward step length
 *    - Maximum backward step length
 *    - Minimum lateral inward step length
 *    - Maximum lateral outward step length
 *    - Maximum rotation angle
 *
*/
// ===================================================================================

class CpConstraintsFootsteps

{

    VectorXd Steps_constr;      // [0]: forward step      0.12;
                                // [1]: backward step       0.10;
                                // [2]: lateral inward step  0.10;
                                // [3]: lateral outward step 0.20;
                                // [4]: rotation step        0.55
    int ctrl_horizon;

    public :

    // Partial: Translation
    MatrixXd    Mxy;            // Matrix of constraints
    VectorXd    Bxy;            // Vector of constraints

    // Partial: Rotation
    MatrixXd    Mang;           // Matrix of constraints
    VectorXd    Bang;           // Vector of constraints

    // Global
    MatrixXd    MSetZmp;        // Matrix of constraints
    VectorXd BSetZmp;           // Vector of constraints

    CpConstraintsFootsteps(CpStanceFootPose *CoPref,
                                        int SptFt[],
                                        int CtrlH,
                                   VectorXd S_Cnstr);

    void UpdateFtstpConstraints(CpStanceFootPose *CoPref,
                                             int SptFt[]);

    void SetFootstepsConstraints(VectorXd setptconstraints);

};


// ===================================================================================
/*
 * CpConstraintsOutputZmp : this class encodes the constraints imposed on the
 * ZMP .
 * From the normal and positions of the foot sides with respect to the foot 
 * origin and from the desired footstep position and orientation, it computes
 * the constraints matrices and the bounds vector
*/

// ===================================================================================

class CpConstraintsOutputZmp
{

    // Definition of normal to the edges (4 sides)
    MatrixXd EdgeNormals;
    // Definition of the limits of the distance
    VectorXd EdgePositions;

    // Edge_pos=[0.15; 0.1; 0.15; 0.1];

    public :

    MatrixXd MuAct;
    VectorXd BuAct;

    // Option for the type of LIPM model used
    int CpModelOption;

    // 
    CpConstraintsOutputZmp(    MpcBased_CP_Model *MpcModel,
                            CP_SelectionMatrices *SMx,
                           Discrete_CP_LIP_Model *DM,
                                CpStanceFootPose *CoPRef, 
                                             int CpOption);
    // This method computes the constraints matrix and vector over the entire horizon
    void FindZmpConstraints(     MpcBased_CP_Model *MpcModel,
                              CP_SelectionMatrices *SMx,
                             Discrete_CP_LIP_Model *DM,
                                  CpStanceFootPose *CoPRef);

    // This method computes the constraints matrix and vector over the first two steps only 
    void FindReducedZmpConstraints12(    MpcBased_CP_Model *MpcModel,
                                      CP_SelectionMatrices *SMx,
                                     Discrete_CP_LIP_Model *DM,
                                          CpStanceFootPose *CoPRef);
    // this method is used to set the positions of the support foot sides with respect to the 
    // foot origin.
    void SetSupportEdgesLimits(VectorXd Edge_lim);

    // this method is used to set the normal to the foot sides with respect to the 
    // foot origin.
    void SetSupportEdgesNormals(MatrixXd Edge_Nor);

};

#endif // MPCOptimizer_H
