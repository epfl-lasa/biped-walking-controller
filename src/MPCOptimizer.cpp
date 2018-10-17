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

#include "TemplateModels.h"
#include "MPCOptimizer.h"

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

CP_QPSolver_OASES::CP_QPSolver_OASES(){}

CP_QPSolver_OASES::~CP_QPSolver_OASES(){}

void CP_QPSolver_OASES::InitialiseQPSol(      MatrixXd H,
                                              VectorXd f,
                                         MatrixXd A_cons,
                                               VectorXd b)
{
       //intitialisation
       uX = 0;
       uY = 0;
       RelCoPXY_n1.resize(6);
       RelCoPXY_n1.setZero(6);

       OrienCoP_n1.resize(3);
       OrienCoP_n1.setZero(3);

       RelCoPXY.resize(6);
       RelCoPXY.setZero(6);
       RelCoPXY(1) = -0.1;

       RelEoSXY.resize(2);
       RelEoSXY.setZero(2);

       uR = 0;

       OrienCoP.resize(3);
       OrienCoP.setZero(3);

       uX_n1 = 0.;
       uY_n1 = 0.;

       OasesSolver = SQProblem (H.rows(), A_cons.rows());

       real_t xOpt[H.rows()];

       /* Setup data of first QP. */
           real_t HMx[H.rows() * H.cols()];
           real_t gVc[H.rows()];
           real_t AMx[A_cons.rows() * A_cons.cols()];
           real_t ubA[b.rows()];
           //

           int iter = 0;

           for (int i=0; i<H.rows(); i++)
           {
               for (int j=0; j<H.cols(); j++)
               {
                   HMx[iter] = H(i,j);
                   iter +=1;
               }
               gVc[i] = f(i);
           }
           iter = 0;

           for (int i=0; i<A_cons.rows(); i++)
           {
               for (int j=0; j<A_cons.cols(); j++)
               {
                   AMx[iter] = A_cons(i,j);
                   iter +=1;
               }
               ubA[i] = b(i);
           }

           nbWorkSetRecal = 450;  //200

           OasesSolver.init(HMx,gVc,AMx,0,0,0,ubA, nbWorkSetRecal, 0);

           OasesSolver.getPrimalSolution(xOpt);


};

 VectorXd CP_QPSolver_OASES::qpOasesSolver (      MatrixXd H,
                                                    VectorXd f,
                                               MatrixXd A_cons,
                                                    VectorXd b)
 {
       //
       VectorXd OptimalSol(H.rows());
       real_t xOpt[H.rows()];

       /* Setup data of first QP. */
           real_t HMx_new[H.rows()*H.cols()];
           real_t gVc_new[H.rows()];
           real_t AMx_new[A_cons.rows()*A_cons.cols()];
           real_t ubA_new[b.rows()];

           int iter;
           iter = 0;
           for (int i=0; i<H.rows(); i++)
           {
               for (int j=0; j<H.cols(); j++)
               {
                   HMx_new[iter] = H(i,j);
                   iter +=1;
               }
               gVc_new[i] = f(i);
           }
           iter = 0;

           for (int i=0; i<A_cons.rows(); i++)
           {
               for (int j=0; j<A_cons.cols(); j++)
               {
                   AMx_new[iter] = A_cons(i,j);
                   iter +=1;
               }
               ubA_new[i] = b(i);
           }


           OasesSolver.hotstart(HMx_new,gVc_new,AMx_new, 0, 0, 0, ubA_new, nbWorkSetRecal, 0);

          //extracting the optimal solution

           OasesSolver.getPrimalSolution(xOpt);

           for (int i=0; i<H.rows(); i++)
           {
               OptimalSol(i) = xOpt[i];
           }

        return OptimalSol;

};

void CP_QPSolver_OASES::setnbWorkingSetRecalculation(int nWSR)
{
     nbWorkSetRecal = nWSR;
}

void CP_QPSolver_OASES::setSolverOptions(Options optionToSet)
{
    OasesSolver.setOptions(optionToSet);
}


   void CP_QPSolver_OASES::CP_QPSolution_Oases(   MatrixXd Q, // Hessian matrix
                                                VectorXd p_k, // gradient vector
                                                  MatrixXd M, // Matrix of constraints
                                                 VectorXd gam) // vector of constraints
   {
       // saving the results one sample before
       RelCoPXY_n1 = RelCoPXY;
       OrienCoP_n1 = OrienCoP;


       VectorXd Optim_uk = CP_QPSolver_OASES::qpOasesSolver(Q,p_k,M,gam);

       int nu = (Optim_uk.rows()-11)/3; //(length(Optim_uk)-9)/3;    # 9: 3 * 3 future steps (pos X,y and orientation)

       // first X optimal control action to apply
       uX    = Optim_uk(0);
       // first Y optimal control action to apply
       uY    = Optim_uk(nu);
       // first angular optimal control action to apply
       uR    = Optim_uk(2*nu+6+2);

       // Optimal action over the prediction horizon
       uXnu.resize(nu);
       uYnu.resize(nu);
       uRnu.resize(nu);

       for (int i=0; i<nu; i++)
       {

           uXnu(i) = Optim_uk(i);

           uYnu(i) = Optim_uk(nu+i);

           uRnu(i) = Optim_uk(2*nu+6+i);
       }

       // Optimal relative footsteps placements
           RelCoPXY.resize(6);
           OrienCoP.resize(3);
       for (int i=0; i<6; i++)
       {
           RelCoPXY(i) = Optim_uk(2*nu+i);
       }
       for (int i=0; i<3; i++)
       {
           OrienCoP(i) = Optim_uk(3*nu+6+2+i);
       }

       RelEoSXY(0) = Optim_uk(2*nu+6);
       RelEoSXY(1) = Optim_uk(2*nu+6+1);

   };

   void CP_QPSolver_OASES::CP_QPSolution_Oases_Ang(   MatrixXd Q,      // Hessian matrix
                                                    VectorXd p_k, // gradient vector
                                                     MatrixXd M,      // Matrix of constraints
                                                    VectorXd gam)  // vector of constraints
   {
       // saving the results one sample before
       RelCoPXY_n1 = RelCoPXY;
       OrienCoP_n1 = OrienCoP;


       VectorXd Optim_uk = CP_QPSolver_OASES::qpOasesSolver(Q,p_k,M,gam);


       int nu = (Optim_uk.rows()-3); // - 3 future steps (pos X,y and orientation)

       // first angular optimal control action to apply
       uR    = Optim_uk(0);

       // Optimal action over the prediction horizon

       uRnu.resize(nu);

       for (int i=0; i<nu; i++)
       {
           uRnu(i) = Optim_uk(i);
       }

       // Optimal relative footsteps orientation
           OrienCoP.resize(3);

       for (int i=0; i<3; i++)
       {
           OrienCoP(i) = Optim_uk(nu+i);
       }

   };

   void CP_QPSolver_OASES::CP_QPSolution_Oases_XY(   MatrixXd Q,      // Hessian matrix
                                                    VectorXd p_k,     // gradient vector
                                                      MatrixXd M,     // Matrix of constraints
                                                     VectorXd gam)    // vector of constraints

{
    // saving the results one sample before
    RelCoPXY_n1 = RelCoPXY;
    OrienCoP_n1 = OrienCoP;


    VectorXd Optim_uk = CP_QPSolver_OASES::qpOasesSolver(Q,p_k,M,gam);

    int nu = (Optim_uk.rows()-8)/2; //(length(Optim_uk)-9)/3;    # 8: 2 * 3 future steps (pos X,y and orientation) + 2 Eos

    // first X optimal control action to apply
    uX    = Optim_uk(0);
    // first Y optimal control action to apply
    uY    = Optim_uk(nu);
    // first angular optimal control action to apply
    uR    = 0.0;

    // Optimal action over the prediction horizon
    uXnu.resize(nu);
    uYnu.resize(nu);
    uRnu.resize(nu);

    for (int i=0; i<nu; i++)
    {

        uXnu(i) = Optim_uk(i);

        uYnu(i) = Optim_uk(nu+i);

        uRnu(i) = 0.0;
    }

    // Optimal relative footsteps placements
        RelCoPXY.resize(6);
        OrienCoP.resize(3);
    for (int i=0; i<6; i++)
    {
        RelCoPXY(i) = Optim_uk(2*nu+i);
    }
    for (int i=0; i<3; i++)
    {
        OrienCoP(i) = 0.;
    }

    RelEoSXY(0) = Optim_uk(2*nu+6);
    RelEoSXY(1) = Optim_uk(2*nu+6+1);

};

   void CP_QPSolver_OASES::CP_QPSolution_Oases_1T(   MatrixXd Q,      // Hessian matrix
                                                VectorXd p_k, // gradient vector
                                                    MatrixXd M,      // Matrix of constraints
                                                VectorXd gam) // vector of constraints

{
    // saving the results one sample before
    RelCoPXY_n1 = RelCoPXY;
    OrienCoP_n1 = OrienCoP;


    VectorXd Optim_uk = -Q.inverse() * p_k ; //CP_QPSolver_Hild::QPhild(Q,p_k,M,gam);

    int nu = (Optim_uk.rows()-4); // 4: 1 * 3 future steps (pos X,y and orientation) +1* Eos

    // first X optimal control action to apply
    uX    = Optim_uk(0);
    // first Y optimal control action to apply

    // Optimal action over the prediction horizon
    uXnu.resize(nu);

    for (int i=0; i<nu; i++)
    {

        uXnu(i) = Optim_uk(i);

    }

    // Optimal relative footsteps placements
        RelCoPXY.resize(6);
        OrienCoP.resize(3);
    for (int i=0; i<3; i++)
    {
        RelCoPXY(i) = Optim_uk(nu+i);
    }
    RelEoSXY(0) = Optim_uk(nu+3);

};


// ===================================================================================
/*
 * CpQMatrix : This class encodes the Hessian matrix related to the MPC QP problem
*/
// ===================================================================================

CpQMatrix::CpQMatrix(MpcBased_CP_Model *MpcModel,
                    double GainsArray[], int CpOption)
   {

       // weights to penalize different objective of the LMPC walkin
       // problem

       for (int i=0; i<8; i++)
       {
           GainsVec[i] = GainsArray[i];
       }


       double alp = GainsVec[0]; // Uact : Control action
       double bet = GainsVec[1]; // IVel : Instantaneous velocity
       double gam = GainsVec[2]; // Ztrk : Output tracking
       double kap = GainsVec[3]; // AVel : Average velocity tracking

       double alp_R = GainsVec[4]; // Uact : Control action
       double bet_R = GainsVec[5]; // IVel : Instantaneous velocity
       double gam_R = GainsVec[6]; // Ztrk : Output tracking
       double kap_R = GainsVec[7]; // AVel : Average velocity tracking

       CpModelOption = CpOption;


       MatrixXd Eye_Nu;
       Eye_Nu.resize(MpcModel->Nu,MpcModel->Nu);
       Eye_Nu.setIdentity(MpcModel->Nu,MpcModel->Nu);

       // Prefactorisation of some term of the Q matrix

       q11trl_x = alp * (MpcModel->TheP).transpose()*MpcModel->TheP
                + gam * (MpcModel->PEu).transpose() * MpcModel->PEu
                + kap * (MpcModel->Ppu).transpose() * (MpcModel->Emx).transpose() * MpcModel->Emx * MpcModel->Ppu;
                //+ bet * (MpcModel->Pvu).transpose() * MpcModel->Pvu;

       q11trl_y = alp * (MpcModel->TheP).transpose()*MpcModel->TheP
                + gam * (MpcModel->PEu).transpose() * MpcModel->PEu
                + kap * (MpcModel->Ppu).transpose() * (MpcModel->Emx).transpose() * MpcModel->Emx * MpcModel->Ppu;
                //+ bet * (MpcModel->Pvu).transpose() * MpcModel->Pvu;

       //cout << "Q11 is :\n"<< q11trl_x << endl;
       if (CpModelOption == 2)
         {
           q11trl_x =   alp * Eye_Nu
                    + bet * (MpcModel->Pvu).transpose() * MpcModel->Pvu
                    + gam * (MpcModel->PEu).transpose() * MpcModel->PEu
                    + kap * (MpcModel->Ppu).transpose() * (MpcModel->Emx).transpose() * MpcModel->Emx * MpcModel->Ppu;

           q11trl_y =   alp * Eye_Nu
                    + bet * (MpcModel->Pvu).transpose() * MpcModel->Pvu
                    + gam * (MpcModel->PEu).transpose() * MpcModel->PEu
                    + 4.0* kap * (MpcModel->Ppu).transpose() * (MpcModel->Emx).transpose() * MpcModel->Emx * MpcModel->Ppu;
         }


       q11Ang = alp_R * Eye_Nu
                + gam_R * (MpcModel->PzuR).transpose() * MpcModel->PzuR
                + kap_R * (MpcModel->PpuR).transpose() * (MpcModel->Emx).transpose() * MpcModel->Emx * MpcModel->PpuR
                + bet_R * (MpcModel->PvuR).transpose() * MpcModel->PvuR;

   }


 MatrixXd CpQMatrix::getQMx(MpcBased_CP_Model *MpcModel,
                               CP_SelectionMatrices *SMx)
 {
     // Hessian mAtrix components

         MatrixXd Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33;

         // Hessian Matrix
             Q12 = -GainsVec[2]* (MpcModel->PEu).transpose() * SMx->Fep_t * SMx->CP_LocalFutureSel;
             Q13 = -GainsVec[2]* (MpcModel->PEu).transpose() * SMx->FeE_t;
             Q21 =  Q12.transpose();
             Q22 =  GainsVec[2]* (SMx->CP_LocalFutureSel).transpose()*(SMx->Fep_t).transpose()* SMx->Fep_t * SMx->CP_LocalFutureSel;
             Q23 =  GainsVec[2]* (SMx->CP_LocalFutureSel).transpose()*(SMx->Fep_t).transpose()* SMx->FeE_t;
             Q31 =  Q13.transpose();
             Q32 =  Q23.transpose();
             Q33 =  GainsVec[2]*(SMx->FeE_t).transpose()* SMx->FeE_t;

             MQx.resize(q11trl_x.rows() + Q21.rows() + Q31.rows(), q11trl_x.cols() + Q12.cols() + Q13.cols());
             MQx.setZero(q11trl_x.rows() + Q21.rows() + Q31.rows(), q11trl_x.cols() + Q12.cols() + Q13.cols());

             MQx.block(0, 0, q11trl_x.rows(), q11trl_x.cols()) = q11trl_x;
             MQx.block(0, q11trl_x.cols(), Q12.rows(), Q12.cols()) = Q12;
             MQx.block(0, q11trl_x.cols()+Q12.cols(), Q13.rows(), Q13.cols()) = Q13;
             MQx.block(q11trl_x.rows(), 0, Q21.rows(), Q21.cols()) = Q21;
             MQx.block(q11trl_x.rows(), Q21.cols(), Q22.rows(), Q22.cols()) = Q22;
             MQx.block(q11trl_x.rows(), Q21.cols()+Q22.cols(), Q23.rows(), Q23.cols()) = Q23;
             MQx.block(q11trl_x.rows()+Q21.rows(), 0, Q31.rows(), Q31.cols()) = Q31;
             MQx.block(q11trl_x.rows()+Q21.rows(), Q31.cols(), Q32.rows(), Q32.cols()) = Q32;
             MQx.block(q11trl_x.rows()+Q21.rows(), Q31.cols()+Q32.cols(), Q33.rows(), Q33.cols()) = Q33;

             MQy = MQx;

             return MQx;


 }

 MatrixXd CpQMatrix::getQMxy(MpcBased_CP_Model *MpcModel,
                               CP_SelectionMatrices *SMx)
 {
     MatrixXd Q12x, Q13x, Q21x, Q22x, Q23x, Q31x, Q32x, Q33x;


     Q12x = -GainsVec[2] * (MpcModel->PEu).transpose() * SMx->GlobalFuture_Fep_t_X;  // g
     Q13x = -GainsVec[2] * (MpcModel->PEu).transpose() * SMx->GlobalFuture_FeE_t_X;
     Q21x =  Q12x.transpose();
     Q22x =  GainsVec[2] * (SMx->GlobalFuture_Fep_t_X).transpose() * SMx->GlobalFuture_Fep_t_X;
     Q23x =  GainsVec[2] * (SMx->GlobalFuture_Fep_t_X).transpose() * SMx->GlobalFuture_FeE_t_X;
     Q31x =  Q13x.transpose();
     Q32x =  Q23x.transpose();
     Q33x =  GainsVec[2] * (SMx->GlobalFuture_FeE_t_X).transpose() * SMx->GlobalFuture_FeE_t_X;


     MatrixXd Q12y, Q13y, Q21y, Q22y, Q23y, Q31y, Q32y, Q33y;


     Q12y = -GainsVec[2] * (MpcModel->PEu).transpose() * SMx->GlobalFuture_Fep_t_Y;
     Q13y = -GainsVec[2] * (MpcModel->PEu).transpose() * SMx->GlobalFuture_FeE_t_Y;
     Q21y =  Q12y.transpose();
     Q22y =  GainsVec[2] * (SMx->GlobalFuture_Fep_t_Y).transpose() * SMx->GlobalFuture_Fep_t_Y;
     Q23y =  GainsVec[2] * (SMx->GlobalFuture_Fep_t_Y).transpose() * SMx->GlobalFuture_FeE_t_Y;
     Q31y =  Q13y.transpose();
     Q32y =  Q23y.transpose();
     Q33y =  GainsVec[2] * (SMx->GlobalFuture_FeE_t_Y).transpose() * SMx->GlobalFuture_FeE_t_Y;

     //
     int nup, nft, nEu;

         nup = q11trl_x.cols();     // number of Columns of q11trl_x
         nft = Q12x.cols();       // number of Columns of Q12x
         nEu = Q13x.cols();       // number of Columns of Q13x


     // x and y Grouped

     MQxy.resize(2*nup+nft+nEu, 2*nup+nft+nEu);
     MQxy.setZero(2*nup+nft+nEu, 2*nup+nft+nEu);

     MQxy.block(0, 0, nup, nup) = q11trl_x;
     MQxy.block(0, 2*nup, nup, nft) = Q12x;
     MQxy.block(0, 2*nup+nft, nup, nEu) = Q13x;

     MQxy.block(nup, nup, nup, nup) = q11trl_y;
     MQxy.block(nup, 2*nup, nup, nft) = Q12y;
     MQxy.block(nup, 2*nup+nft, nup, nEu) = Q13y;

     MQxy.block(2*nup, 0, nft, nup) = Q21x;
     MQxy.block(2*nup, nup, nft, nup) = Q21y;
     MQxy.block(2*nup,2*nup, nft, nft) = Q22x+Q22y;
     MQxy.block(2*nup, 2*nup+nft, nft, nEu) = Q23x+Q23y;

     MQxy.block(2*nup+nft, 0, nEu, nup) = Q31x;
     MQxy.block(2*nup+nft, nup, nEu, nup) = Q31y;
     MQxy.block(2*nup+nft, 2*nup, nEu, nft) = Q32x+Q32y;
     MQxy.block(2*nup+nft, 2*nup+nft, nEu, nEu) = Q33x+Q33y;

     return MQxy;

 }

 MatrixXd CpQMatrix::getQMang(MpcBased_CP_Model *MpcModel,
                               CP_SelectionMatrices *SMx)
 {
     int n2, // number of Columns of MpcModel->PzuR
         nt; // number of Columns of Uk_ang

         n2 = (MpcModel->PzuR).cols();
         nt = (SMx->LocalFutureSel).cols();

     // angular
      Qang.resize(n2+nt, n2+nt);
      Qang.setZero(n2+nt, n2+nt);

//      Qang.block(0, 0, q11Ang.rows(), q11Ang.cols()) = q11Ang;
//      Qang.block(0, n2, (-GainsVec[2] * (MpcModel->PzuR).transpose() * SMx->LocalFutureSel).rows(),
//                        (-GainsVec[2] * (MpcModel->PzuR).transpose() * SMx->LocalFutureSel).cols()) = -GainsVec[2] * (MpcModel->PzuR).transpose() * SMx->LocalFutureSel;
//      Qang.block(n2, 0, (-GainsVec[2] * (SMx->LocalFutureSel).transpose() * MpcModel->PzuR).rows(),
//                        (-GainsVec[2] * (SMx->LocalFutureSel).transpose() * MpcModel->PzuR).cols()) = -GainsVec[2] * (SMx->LocalFutureSel).transpose() * MpcModel->PzuR;
//      Qang.block(n2, n2,( GainsVec[2] * (SMx->LocalFutureSel).transpose() * SMx->LocalFutureSel).rows(),
//                        ( GainsVec[2] * (SMx->LocalFutureSel).transpose() * SMx->LocalFutureSel).cols()) = GainsVec[2] * (SMx->LocalFutureSel).transpose() * SMx->LocalFutureSel;
      MatrixXd Q12;
      Q12 = -GainsVec[6] * (MpcModel->PzuR).transpose() * SMx->LocalFutureSel;
      Qang.block(0, 0, n2, n2) =  q11Ang;
      Qang.block(0, n2, n2,nt) = Q12;
      Qang.block(n2, 0, nt,n2) = Q12.transpose();
      Qang.block(n2, n2,nt,nt) =  GainsVec[6] * (SMx->LocalFutureSel).transpose() * SMx->LocalFutureSel;

      return Qang;

 }

 void CpQMatrix::QmxElements(MpcBased_CP_Model *MpcModel,
                                 CP_SelectionMatrices *SMx)
 {

     MatrixXd Qxy, Q_ang;

     Qxy = CpQMatrix::getQMxy( MpcModel,
                                     SMx);

     Q_ang = CpQMatrix::getQMang( MpcModel,
                                     SMx);


        // Overall matrix
        MxQk.resize(Qxy.rows() + Q_ang.rows(), Qxy.cols() + Q_ang.cols());
        MxQk.setZero(Qxy.rows() + Q_ang.rows(), Qxy.cols() + Q_ang.cols());
        MxQk.block(0, 0, Qxy.rows(), Qxy.cols()) = Qxy;
        MxQk.block(Qxy.rows(), Qxy.cols(), Q_ang.rows(), Q_ang.cols()) = Q_ang;

        //std::cout<<" Qang - Qang.transpose() is:\n"<< q11Ang - q11Ang.transpose() << std::endl;

}



// ===================================================================================
/*
 * CpPVectorQP : This class encodes the Gradient Vector related to the MPC QP problem
*/
// ===================================================================================

CpPVectorQP::CpPVectorQP(MpcBased_CP_Model *MpcModel,
                     double GainsArray[], int CpOption)

   {

         // weights to penalize different objectives of the LMPC walking
         // problem

         for (int i=0; i<8; i++)
         {
            GainsVec[i] = GainsArray[i];
         }

            //double alp = GainsVec[0]; // Uact : Control action
            double bet_R = GainsVec[5]; // IVel : Instantaneous velocity
            double gam_R = GainsVec[6]; // Ztrk : Output tracking
            double kap_R = GainsVec[7]; // AVel : Average velocity tracking

            // Type of LIPM model
            CpModelOption = CpOption;

         // Prefactorisation of some term of the P vector
            // P1trl = + gam * (MpcModel->PEu).transpose() * MpcModel->PEs;
                    //+ kap * (MpcModel->Ppu).transpose() * (MpcModel->getEmx()).transpose() * MpcModel->getEmx() * MpcModel->Pps
                    //+ bet * (MpcModel->Pvu).transpose() * MpcModel->Pvs;

         // theta
            P1Ang = bet_R * (MpcModel->PvuR).transpose() * MpcModel->PvsR
                    + gam_R * (MpcModel->PzuR).transpose() * MpcModel->PzsR;
                    + kap_R * (MpcModel->PpuR).transpose() * (MpcModel->Emx).transpose() * MpcModel->Emx * MpcModel->PpsR;

   }

  VectorXd CpPVectorQP::getVeckx(MpcBased_CP_Model *MpcModel,    // LMPC model of the LIPM
                                   Discrete_CP_LIP_Model *St,      // CoM State variables
                                    CpStanceFootPose *CoPRef,  // Stance feet pose
                                   CP_SelectionMatrices *SMx,    // Selection matrices
                                    VelocitiesSetPoints *VeloRef, // reference velocities wrt. the world frame
                                         VectorXd OptimSolXY_n1)

 {
      // ****************************
      VectorXd VxAbsEmx(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]), // VxAbs over the Current and the 1st Future step
               VyAbsEmx(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]); // WzAbs over the Current and the 1st Future step

      for ( int i= 0; i<(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]); i++)
      {
              VxAbsEmx(i) = (VeloRef->VxAbs)(i);
              VyAbsEmx(i) = (VeloRef->VyAbs)(i);
      }

      // Gradient vector
      VectorXd Pk1, Pk2, Pk3, fpx;
      // Gradient vector

      fpx = SMx->Fep_t * SMx->CP_LocalCurrentSel* CoPRef->CoPRefX - MpcModel->PEs * (St->StatesX + St->StatesDistX);

      if (CpModelOption == 0||CpModelOption == 1)
      {
          Pk1 = -GainsVec[2]* (MpcModel->PEu).transpose() * fpx
                -GainsVec[0]* (MpcModel->TheP).transpose()* MpcModel->e1 * OptimSolXY_n1(0)
                +GainsVec[3]* (MpcModel->Ppu).transpose() * (MpcModel->getEmx()).transpose() * (MpcModel->getEmx() * MpcModel->Pps * (St->StatesX + St->StatesDistX) - VxAbsEmx);

      }
      else if (CpModelOption == 2)
      {

          Pk1 = -GainsVec[2]* (MpcModel->PEu).transpose() * fpx
                +GainsVec[1]* (MpcModel->Pvu).transpose()* (MpcModel->Pvs * (St->StatesX + St->StatesDistX) - VeloRef->VxAbs)
                +GainsVec[3]* (MpcModel->Ppu).transpose() * (MpcModel->getEmx()).transpose() * (MpcModel->getEmx() * MpcModel->Pps * (St->StatesX + St->StatesDistX) - VxAbsEmx);


      }
          Pk2 = GainsVec[2]* (SMx->CP_LocalFutureSel).transpose()*(SMx->Fep_t).transpose()*fpx;
          Pk3 = GainsVec[2]* (SMx->FeE_t).transpose()* fpx;


          VecPx.resize(Pk1.rows()+Pk2.rows()+Pk3.rows());

          VecPx.segment(0, Pk1.rows()) = Pk1;
          VecPx.segment(Pk1.rows(), Pk2.rows()) = Pk2;
          VecPx.segment(Pk1.rows()+Pk2.rows(), Pk3.rows()) = Pk3;

          VecPy = VecPx;

          return VecPx;


 }

   VectorXd CpPVectorQP::getVeckxy(   MpcBased_CP_Model *MpcModel,       // LMPC model of the LIPM
                                        Discrete_CP_LIP_Model *St,       // CoM State variables
                                         CpStanceFootPose *CoPRef,        // Stance feet pose
                                        CP_SelectionMatrices *SMx,       // Selection matrices
                                     VelocitiesSetPoints *VeloRef,       // reference velocities wrt. the world frame
                                         VectorXd OptimSolXY_n1)

  {
       // ****************************
       VectorXd VxAbsEmx(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]),    // VxAbs over the Current and the 1st Future step
                VyAbsEmx(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]);    // VyAbs over the Current and the 1st Future step

//       for ( int i= 0; i<(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]); i++)
//       {
//               VxAbsEmx(i) = (VeloRef->VxAbs)(i);
//               VyAbsEmx(i) = (VeloRef->VyAbs)(i);
//       }

       VxAbsEmx = (VeloRef->VxAbs).segment(0,MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]);
       VyAbsEmx = (VeloRef->VyAbs).segment(0,MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]);

       //    VectorXd Pk1, Pk2, Pk3, fpx, PK;
           // x
           VectorXd Pk1x, Pk2x, Pk3x, Fpx;
           // y
           VectorXd Pk1y, Pk2y, Pk3y, Fpy;

           // Gradient vector

           // x
           
           if (CpModelOption == 0||CpModelOption == 1)
             {
               // x
               Fpx = SMx->Fep_t * SMx->CP_GlobalCurrentSelX* CoPRef->CoPRefX - MpcModel->PEs * (St->StatesX + St->StatesDistX);

               Pk1x = -GainsVec[2]* (MpcModel->PEu).transpose() * Fpx
                      -GainsVec[0]* MpcModel->TheP_T_e1 * OptimSolXY_n1(0)
                      +GainsVec[3]* MpcModel->Ppu_T_Emx_T * (MpcModel->Emx_Pps * (St->StatesX + St->StatesDistX) - VxAbsEmx);

               Pk2x = GainsVec[2]* (SMx->Fep_t * SMx->CP_GlobalFutureSelX + SMx->FeE_t * SMx->RotX3).transpose() * Fpx;
               Pk3x = GainsVec[2]* (SMx->FeE_t* SMx->RotX33).transpose()* Fpx;
               // y
               Fpy = SMx->Fep_t * SMx->CP_GlobalCurrentSelY* CoPRef->CoPRefY - MpcModel->PEs * (St->StatesY +  St->StatesDistY);

               Pk1y = -GainsVec[2]* (MpcModel->PEu).transpose() * Fpy
                      -GainsVec[0]* MpcModel->TheP_T_e1 * OptimSolXY_n1(1)
                      +GainsVec[3]* MpcModel->Ppu_T_Emx_T * (MpcModel->Emx_Pps * (St->StatesY +  St->StatesDistY) - VyAbsEmx);

               Pk2y = GainsVec[2]* (SMx->Fep_t * SMx->CP_GlobalFutureSelY + SMx->FeE_t * SMx->RotY3).transpose()*Fpy;
               Pk3y = GainsVec[2]* (SMx->FeE_t* SMx->RotY33).transpose()* Fpy;


             }
           else if (CpModelOption == 2)
             {
               // x
               Fpx = SMx->Fep_t * SMx->CP_GlobalCurrentSelX* CoPRef->CoPRefX - MpcModel->PEs * (St->StatesX + St->StatesDistX);

               Pk1x = -GainsVec[2]* (MpcModel->PEu).transpose() * Fpx
                      +GainsVec[1]* (MpcModel->Pvu).transpose()* (MpcModel->Pvs * (St->StatesX + St->StatesDistX) - VeloRef->VxAbs)
                      +GainsVec[3]* MpcModel->Ppu_T_Emx_T * (MpcModel->Emx_Pps * (St->StatesX + St->StatesDistX) - VxAbsEmx);

               Pk2x = GainsVec[2]* (SMx->Fep_t * SMx->CP_GlobalFutureSelX + SMx->FeE_t * SMx->RotX3).transpose() * Fpx;
               Pk3x = GainsVec[2]*(SMx->FeE_t* SMx->RotX33).transpose()* Fpx;
               // y
               Fpy = SMx->Fep_t * SMx->CP_GlobalCurrentSelY* CoPRef->CoPRefY - MpcModel->PEs * (St->StatesY +  St->StatesDistY);

               Pk1y = -GainsVec[2]* (MpcModel->PEu).transpose() * Fpy
                      +GainsVec[1]* (MpcModel->Pvu).transpose()* (MpcModel->Pvs * (St->StatesY +  St->StatesDistY) - VeloRef->VyAbs)
                      +4.0* GainsVec[3]* MpcModel->Ppu_T_Emx_T * (MpcModel->Emx_Pps * (St->StatesY +  St->StatesDistY) - VyAbsEmx);

               Pk2y = GainsVec[2]* (SMx->Fep_t * SMx->CP_GlobalFutureSelY + SMx->FeE_t * SMx->RotY3).transpose()*Fpy;
               Pk3y = GainsVec[2]*(SMx->FeE_t* SMx->RotY33).transpose()* Fpy;

             }

           // Grouping x and y components
           VecPxy.resize(2*Pk1x.rows()+Pk2x.rows()+Pk3x.rows());

           VecPxy.segment(0, Pk1x.rows()) = Pk1x;
           VecPxy.segment(Pk1x.rows(), Pk1y.rows()) = Pk1y;

           VecPxy.segment(2*Pk1x.rows(), Pk2x.rows()) = Pk2x+Pk2y;
           VecPxy.segment(2*Pk1x.rows()+Pk2x.rows(), Pk3x.rows()) = Pk3x+Pk3y;

           return VecPxy;


  }

   VectorXd CpPVectorQP::getVeckAng(  MpcBased_CP_Model *MpcModel,    // LMPC model of the LIPM
                                        Discrete_CP_LIP_Model *St,      // CoM State variables
                                         CpStanceFootPose *CoPRef,  // Stance feet pose
                                        CP_SelectionMatrices *SMx,    // Selection matrices
                                     VelocitiesSetPoints *VeloRef) // reference velocities wrt. the world frame

   {


       // ****************************
       VectorXd  WzAbsEmx(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]); // WzAbs over the Current and the 1st Future step
       VectorXd  LocalCurrentSel_CoPRefR;
                 LocalCurrentSel_CoPRefR = (SMx->LocalCurrentSel * CoPRef->CoPRefR);

//       for ( int i= 0; i<(MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]); i++)
//       {
//              WzAbsEmx(i) = (VeloRef->WzAbs)(i);
//       }
       // Angular component related to the tracking of the ref. CoP orientation

       WzAbsEmx = (VeloRef->WzAbs).segment(0,MpcModel->SamplesPerStep[0]+MpcModel->SamplesPerStep[1]);
       // Angular component related to the tracking of the ref. CoP orientation
       PkAng1 = P1Ang * (St->StatesR + St->StatesDistR) -  GainsVec[5] * (MpcModel->PvuR).transpose() * VeloRef->WzAbs
               - GainsVec[6] * (MpcModel->PzuR).transpose() * LocalCurrentSel_CoPRefR
               - GainsVec[7] * MpcModel->PpuR_T_Emx_T * WzAbsEmx;

       // Angular component related to the generation of CoP ref orientation
       PkAng2 = -GainsVec[6] * (SMx->LocalFutureSel).transpose() * (MpcModel->PzsR * (St->StatesR + St->StatesDistR)
                - (SMx->LocalCurrentSel * CoPRef->CoPRefR));

       // Grouping Angular components
       VecPAng.resize(PkAng1.rows() + PkAng2.rows());
       VecPAng.setZero(PkAng1.rows() + PkAng2.rows());
       VecPAng.segment(0, PkAng1.rows()) = PkAng1;
       VecPAng.segment(PkAng1.rows(), PkAng2.rows()) = PkAng2;

       return VecPAng;

   }


 void CpPVectorQP::PVecElements(MpcBased_CP_Model *MpcModel, // LMPC model of the LIPM
                                  Discrete_CP_LIP_Model *St, // CoM State variables
                                  CpStanceFootPose *CoPRef,  // Stance feet pose
                                  CP_SelectionMatrices *SMx,    // Selection matrices
                                  VelocitiesSetPoints *VeloRef,
                                  VectorXd OptimSolXY_n1) // reference velocities wrt. the world frame

 {


        VectorXd P_xy, P_ang;

        P_xy = CpPVectorQP::getVeckxy(       MpcModel,    // LMPC model of the LIPM
                                                   St,    // CoM State variables
                                               CoPRef,    // Stance feet pose
                                                  SMx,    // Selection matrices
                                              VeloRef,    // reference velocities wrt. the world frame
                                       OptimSolXY_n1);

        P_ang = CpPVectorQP::getVeckAng(     MpcModel,    // LMPC model of the LIPM
                                                   St,    // CoM State variables
                                               CoPRef,    // Stance feet pose
                                                  SMx,    // Selection matrices
                                             VeloRef);   // reference velocities wrt. the world frame


        // Grouping all P vector elements

        VecPk.resize(P_xy.rows() + P_ang.rows());
        VecPk.setZero(P_xy.rows() + P_ang.rows());
        VecPk.segment(0, P_xy.rows()) = P_xy;
        VecPk.segment(P_xy.rows(), P_ang.rows()) = P_ang;

 }


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


CpConstraintsFootsteps::CpConstraintsFootsteps(CpStanceFootPose *CoPref,
                                                              int SptFt[],
                                                                int CtrlH,
                                                         VectorXd S_Cnstr)
 {

      // Control Horizon
      ctrl_horizon = CtrlH;
      // These constraints are set for automatic zmp generation
      //
      Steps_constr = S_Cnstr;
      //
      CpConstraintsFootsteps::UpdateFtstpConstraints(CoPref, SptFt);


 };

 void CpConstraintsFootsteps::UpdateFtstpConstraints(CpStanceFootPose *CoPref,
                                                                  int SptFt[])
 {

      // Translational displacement M matrix
      // *************************************
      MatrixXd MP_Tr, MN_Tr;

      MP_Tr.resize(9,6); // Const Matrix for Positve Translation
      MP_Tr.setZero(9,6);

      MN_Tr.resize(9,6); // Const Matrix for Negative Translation
      MN_Tr.setZero(9,6);

      // for positive velocity (positive translation)
      // X
      MP_Tr(0,0) = 1.; MP_Tr(3,2) = 1.; MP_Tr(6,4) = 1.; // x
      // Y
      MP_Tr(1,1) =  1.; MP_Tr(4,3) =  1.; MP_Tr(7,5) =  1.;
      MP_Tr(2,1) = -1.; MP_Tr(5,3) = -1.; MP_Tr(8,5) = -1.;

      // for Negative velocity (negative translation)
      // X
      MN_Tr(0,0) = -1.; MN_Tr(3,2) = -1.; MN_Tr(6,4) = -1.; // x
      // Y
      MN_Tr(1,1) =  1.; MN_Tr(4,3) =  1.; MN_Tr(7,5) =  1.;
      MN_Tr(2,1) = -1.; MN_Tr(5,3) = -1.; MN_Tr(8,5) = -1.;

      // Matrix of constraints in translation
      int nu = ctrl_horizon;
      Mxy.resize(18, 2*nu + 6);
      Mxy.setZero(18, 2*nu + 6);

      Mxy.block(0, 2*nu, MP_Tr.rows(), MP_Tr.cols()) = MP_Tr; // with +ve velocity
      Mxy.block(9, 2*nu, MN_Tr.rows(), MN_Tr.cols()) = MN_Tr; // with -ve velocity

      //
      // B matrix
      //
      VectorXd bx(9), b1y(9), b2y(9),
                  by_1(9), by_2(9);

      //  bx1
      bx.setZero(9); // [1;0;0;1;0;0;1;0;0];
      bx(0) = 1.; bx(3) = 1.; bx(6) = 1.;
      // by_1 and by_2
      by_1.setZero(9);
      by_2.setZero(9);
      by_1(2) = 1.; by_1(4) = 1.; by_1(8) = 1.; //[0;0;1;0;1;0;0;0;1]
      by_2(1) = 1.; by_2(5) = 1.; by_2(7) = 1.; //[0;1;0;0;0;1;0;1;0]

      b1y  = by_1 * SptFt[1] + // right stance Uc_k_0
             by_2 * SptFt[0];   // left stance  Uk_0

      b2y  = by_2 * SptFt[1] + // right stance
             by_1 * SptFt[0];   // left stance

      // vector of constraints in translation
      Bxy.resize(18);
      Bxy.setZero(18);
      Bxy.segment(0, ( bx  * Steps_constr(0)
                       -b1y * Steps_constr(2)
                       +b2y * Steps_constr(3)).rows()) = ( bx  * Steps_constr(0)      // x forward
                                                           -b1y * Steps_constr(2)     // y inward
                                                           +b2y * Steps_constr(3));   // y outward

      Bxy.segment(9, ( bx  * Steps_constr(1)
                          -b1y * Steps_constr(2)
                          +b2y * Steps_constr(3)).rows()) = ( bx  * Steps_constr(1)       // x backward
                                                              -b1y * Steps_constr(2)      // y inward
                                                              +b2y * Steps_constr(3));    // y outward

      // Constraints on angular displacement
      // ************************************
      //
      // M Matrix
      //
      MatrixXd MP_Rt;

      MP_Rt.resize(3,3);
      MP_Rt.setZero(3,3);
      MP_Rt.setIdentity(3,3);
      MP_Rt(1,0) = -1.; MP_Rt(2,1) = -1;

      Mang.resize(6,nu+3);
      Mang.setZero(6,nu+3);
      Mang.block(0, nu, MP_Rt.rows(), MP_Rt.cols()) = MP_Rt;           // theta  with +ve velocity
      Mang.block(3, nu, (-MP_Rt).rows(), (-MP_Rt).cols()) = -MP_Rt;    // theta  with -ve velocity
      //
      // B vector
      //
      VectorXd b1ang(3), b2ang(3);
      b1ang.setOnes(3);
      b2ang.setZero(3);
      b2ang(0) = 1.;
      b2ang = b2ang * CoPref->CoPRefR;

      // vector of constraints in rotation
      Bang.resize(6);
      Bang.setZero(6);

      Bang.segment(0, (Steps_constr(4) * b1ang + b2ang).rows()) = (Steps_constr(4) * b1ang + b2ang);
      Bang.segment(3, (Steps_constr(4) * b1ang - b2ang).rows()) = (Steps_constr(4) * b1ang - b2ang);

      // Overall Constraints
      //*********************
      // M MATRIX
      //
      MSetZmp.resize(18+6, 2*nu+nu+6+2+3);
      MSetZmp.setZero(18+6, 2*nu+nu+6+2+3);

      MSetZmp.block(0, 0, Mxy.rows(), Mxy.cols()) = Mxy;
      MSetZmp.block(18, 2*nu+6+2, Mang.rows(), Mang.cols()) = Mang; // matrix M

      // B MATRIX
      BSetZmp.resize(18+6);
      BSetZmp.setZero(18+6);

      BSetZmp.segment(0, Bxy.rows()) = Bxy;
      BSetZmp.segment(18, Bang.rows()) = Bang;


 };

 void CpConstraintsFootsteps::SetFootstepsConstraints(VectorXd setptconstraints)
 {
     Steps_constr = setptconstraints;
 }


   // ===================================================================================
/*
 * CpConstraintsOutputZmp : this class encodes the constraints imposed on the
 * ZMP .
 * From the normal and positions of the foot sides with respect to the foot 
 * origin and from the desired footstep position and orientation, it computes
 * the constraints matrices and the bounds vector
*/

// ===================================================================================

CpConstraintsOutputZmp::CpConstraintsOutputZmp(MpcBased_CP_Model *MpcModel,
                                                    CP_SelectionMatrices *SMx,
                                                    Discrete_CP_LIP_Model *DMod,
                                                     CpStanceFootPose *CoPRef, int CpOption)
{
        // Initialisation of the normal
        EdgeNormals.resize(2,4);
        EdgeNormals.setZero(2,4);

        EdgeNormals(0,0) = 1.; EdgeNormals(1,1) = 1.; //[1  0  -1  0;
        EdgeNormals(0,2) =-1.; EdgeNormals(1,3) =-1.; //0  1   0 -1];

        CpModelOption = CpOption;

        // Initialisation of the edges positions
        EdgePositions.resize(4);
        // [0.050; 0.01; 0.03; 0.01];
//        EdgePositions(0) = 0.050; EdgePositions(1) = 0.12; //0.02;
//        EdgePositions(2) = 0.030; EdgePositions(3) = 0.12; //0.02;

        EdgePositions(0) = 0.070; EdgePositions(1) = 0.03; //0.02;
        EdgePositions(2) = 0.070; EdgePositions(3) = 0.03; //0.02;

        CpConstraintsOutputZmp::FindZmpConstraints(MpcModel,
                                                        SMx,
                                                         DMod,
                                                     CoPRef);




};

void CpConstraintsOutputZmp::FindZmpConstraints(MpcBased_CP_Model *MpcModel,
                                                      CP_SelectionMatrices *SMx,
                                                      Discrete_CP_LIP_Model *DMod,
                                                       CpStanceFootPose *CoPRef)
{


       int n0, n1, n2, nu, nbs, nuk;

           n0  = (MpcModel->SamplesPerStep)[0]; //nb of samples in Step1
           n1  = (MpcModel->SamplesPerStep)[1]; //nb of samples in Step2
           n2  = (MpcModel->SamplesPerStep)[2]; //nb of samples in Step3  4

           nu  = (MpcModel->PEu).cols(); // nb of columns of Pzu matrix
           nbs = EdgeNormals.cols();  // nb of vertices of support foot
           nuk = (SMx->GlobalFutureSelX).cols(); // nb of future steps

           // Stance Foot Rotation matrix over four steps
           MatrixXd Rot_theta;
           Rot_theta.resize(8,2);
           Rot_theta.setZero(8,2);
           // current stance foot
           Rot_theta(0,0) =  cos((SMx->Theta_k1)(0));
           Rot_theta(0,1) = -sin((SMx->Theta_k1)(0));
           Rot_theta(1,0) =  sin((SMx->Theta_k1)(0));
           Rot_theta(1,1) =  cos((SMx->Theta_k1)(0));
           // first predicted step
           Rot_theta(2,0) =  cos((SMx->Theta_k1)(n0));
           Rot_theta(2,1) = -sin((SMx->Theta_k1)(n0));
           Rot_theta(3,0) =  sin((SMx->Theta_k1)(n0));
           Rot_theta(3,1) =  cos((SMx->Theta_k1)(n0));
           // second predicted step
           Rot_theta(4,0) =  cos((SMx->Theta_k1)(n0+n1));
           Rot_theta(4,1) = -sin((SMx->Theta_k1)(n0+n1));
           Rot_theta(5,0) =  sin((SMx->Theta_k1)(n0+n1));
           Rot_theta(5,1) =  cos((SMx->Theta_k1)(n0+n1));
           // third predicted step
           Rot_theta(6,0) =  cos((SMx->Theta_k1)(n0+n1+n2));
           Rot_theta(6,1) = -sin((SMx->Theta_k1)(n0+n1+n2));
           Rot_theta(7,0) =  sin((SMx->Theta_k1)(n0+n1+n2));
           Rot_theta(7,1) =  cos((SMx->Theta_k1)(n0+n1+n2));

           // normal to the support polygon (foot) vertices over the
           // prediction horizon (four footsteps)
           MatrixXd dxy_0;
           dxy_0 = Rot_theta * EdgeNormals;


           /* Product of Rotated X and Y components of normals to
            * support polygon vertices with EACH SAMPLE of the matrices
            * Pzu, Uk_x, Uk_y, and vector (Uc_k*Xc_k_h - Pzs*x_k_h)
            * (h=x,y respectively).
            *
            * X components of rotated normals over the four steps
            *     (dxy_0.row(1)) : current step (step1)
            *     (dxy_0.row(3)) : first future step (step2)
            *     (dxy_0.row(5)) : second future step (step3)
            *     (dxy_0.row(7)) : third future step (step4)
            *
            *
            * Y components of rotated normals over the four steps
            *     (dxy_0.row(2)) : current step (step1)
            *     (dxy_0.row(4)) : first future step (step2)
            *     (dxy_0.row(6)) : second future step (step3)
            *     (dxy_0.row(8)) : third future step (step4)
            *
            * ******************************************************** */

               MatrixXd DX_theta, DX_Uk_x,
                        DY_theta, DY_Uk_y;

               VectorXd DX_S_Ux0, DY_S_Uy0;

               // Initialisation
               //***************
               // combination matrix of the jerk and the constraints
               // Product with Matrix Pzu
               DX_theta.resize(nbs*(n0+n1+n2+n2),nu);
               DX_theta.setZero(nbs*(n0+n1+n2+n2),nu);

               DY_theta.resize(nbs*(n0+n1+n2+n2),nu);
               DY_theta.setZero(nbs*(n0+n1+n2+n2),nu);

               // combination matrix of relative footsteps to the constraints
               // Product with Matrix Uk_x and Uk_y
               DX_Uk_x.resize(nbs*(n0+n1+n2+n2),nuk);
               DX_Uk_x.setZero(nbs*(n0+n1+n2+n2),nuk);

               DY_Uk_y.resize(nbs*(n0+n1+n2+n2),nuk);
               DY_Uk_y.setZero(nbs*(n0+n1+n2+n2),nuk);

               // constraints of ZMP due to the state of the CoM
               // Product with Matrix
               DX_S_Ux0.resize(nbs*(n0+n1+n2+n2),1);
               DX_S_Ux0.setZero(nbs*(n0+n1+n2+n2),1);

               DY_S_Uy0.resize(nbs*(n0+n1+n2+n2),1);
               DY_S_Uy0.setZero(nbs*(n0+n1+n2+n2),1);

               // vector of Zmp constraints due to the state of the CoM and
               // the current stance foot position
               VectorXd S_Ux0, S_Uy0;

               if (CpModelOption == 0 || CpModelOption == 1)
                 {
                   S_Ux0 = SMx->GlobalCurrentSelX * CoPRef->CoPRefX; // Uckx_fcx = Uc_k_x*xk_fc_x;
                   S_Uy0 = SMx->GlobalCurrentSelY * CoPRef->CoPRefY;
                 }
               else if (CpModelOption == 2)
                 {
                   S_Ux0 = SMx->GlobalCurrentSelX * CoPRef->CoPRefX - MpcModel->Pzs * DMod->StatesX; // Uckx_fcx = Uc_k_x*xk_fc_x;
                   S_Uy0 = SMx->GlobalCurrentSelY * CoPRef->CoPRefY - MpcModel->Pzs * DMod->StatesY;
                 }


               // Computation
               //************

               for (int i=0; i<n0; i++)
               {
                   // DX_Szu and DY_Szu
                   if (CpModelOption == 0 || CpModelOption == 1)
                     {
                       DX_theta.block(nbs*i, i, ((dxy_0.row(0)).transpose()).rows(),
                                                ((dxy_0.row(0)).transpose()).cols()) =  (dxy_0.row(0)).transpose();
                       DY_theta.block(nbs*i, i, ((dxy_0.row(1)).transpose()).rows(),
                                                ((dxy_0.row(1)).transpose()).cols()) =  (dxy_0.row(1)).transpose();
                     }
                   else if (CpModelOption == 2)
                     {
                       for(int j=0; j<nu; j++)
                         {
                           DX_theta.block(nbs*i, j, ((dxy_0.row(0)).transpose()).rows(),
                                                    ((dxy_0.row(0)).transpose()).cols()) =  (dxy_0.row(0)).transpose()*MpcModel->Pzu(i,j);
                           DY_theta.block(nbs*i, j, ((dxy_0.row(1)).transpose()).rows(),
                                                    ((dxy_0.row(1)).transpose()).cols()) =  (dxy_0.row(1)).transpose()*MpcModel->Pzu(i,j);
                         }
                     }



                   for (int j=0; j<nuk; j++)
                   {
                       DX_Uk_x.block(nbs*i, j, ((dxy_0.row(0)).transpose()).rows(),
                                               ((dxy_0.row(0)).transpose()).cols()) =  (dxy_0.row(0)).transpose()*(SMx->GlobalFutureSelX)(i,j);
                       DY_Uk_y.block(nbs*i, j, ((dxy_0.row(1)).transpose()).rows(),
                                               ((dxy_0.row(1)).transpose()).cols()) =  (dxy_0.row(1)).transpose()*(SMx->GlobalFutureSelY)(i,j);
                   }
                   // DX_Szs_Ux0 and DX_Szs_Uy0
                   DX_S_Ux0.segment(nbs*i, ((dxy_0.row(0)).transpose()).rows()) =  (dxy_0.row(0)).transpose()*S_Ux0(i);
                   DY_S_Uy0.segment(nbs*i, ((dxy_0.row(1)).transpose()).rows()) =  (dxy_0.row(1)).transpose()*S_Uy0(i);
               }


               // Step 2
               for (int i=0; i<n1; i++)
               {
                   // DX_Szu and DY_Szu
                   if (CpModelOption == 0 || CpModelOption == 1)
                     {
                       DX_theta.block(nbs*(n0+i), (n0+i),   ((dxy_0.row(2)).transpose()).rows(),
                                                            ((dxy_0.row(2)).transpose()).cols()) =  (dxy_0.row(2)).transpose();
                       DY_theta.block(nbs*(n0+i), (n0+i),   ((dxy_0.row(3)).transpose()).rows(),
                                                            ((dxy_0.row(3)).transpose()).cols()) =  (dxy_0.row(3)).transpose();
                     }
                   else if (CpModelOption == 2)
                     {
                       for(int j=0; j<nu; j++)
                         {
                           DX_theta.block(nbs*(n0+i), (n0+i),   ((dxy_0.row(2)).transpose()).rows(),
                                                                ((dxy_0.row(2)).transpose()).cols()) =  (dxy_0.row(2)).transpose()*MpcModel->Pzu(n0+i,j);
                           DY_theta.block(nbs*(n0+i), (n0+i),   ((dxy_0.row(3)).transpose()).rows(),
                                                                ((dxy_0.row(3)).transpose()).cols()) =  (dxy_0.row(3)).transpose()*MpcModel->Pzu(n0+i,j);
                         }
                     }


                   // DX_Uk_x and DY_Uk_y
                   for (int j=0; j<nuk; j++)
                   {
                       DX_Uk_x.block(nbs*(n0+i), j,  ((dxy_0.row(2)).transpose()).rows(),
                                                     ((dxy_0.row(2)).transpose()).cols()) =  (dxy_0.row(2)).transpose()*(SMx->GlobalFutureSelX)(n0+i,j);
                       DY_Uk_y.block(nbs*(n0+i), j,  ((dxy_0.row(3)).transpose()).rows(),
                                                     ((dxy_0.row(3)).transpose()).cols()) =  (dxy_0.row(3)).transpose()*(SMx->GlobalFutureSelY)(n0+i,j);
                   }
                   // DX_Szs_Ux0 and DX_Szs_Uy0
                   DX_S_Ux0.segment(nbs*(n0+i), ((dxy_0.row(2)).transpose()).rows()) =  (dxy_0.row(2)).transpose()*S_Ux0(n0+i);
                   DY_S_Uy0.segment(nbs*(n0+i), ((dxy_0.row(3)).transpose()).rows()) =  (dxy_0.row(3)).transpose()*S_Uy0(n0+i);
               }



               // step 3  4
               for (int i=0; i<n2; i++)
               {
                   // DX_Szu and DY_Szu
                   if (CpModelOption == 0 || CpModelOption == 1)
                     {
                       // step 3
                       DX_theta.block(nbs*(n0+n1+i), (n0+n1+i),  ((dxy_0.row(4)).transpose()).rows(),
                                                                 ((dxy_0.row(4)).transpose()).cols()) =  (dxy_0.row(4)).transpose();
                       DY_theta.block(nbs*(n0+n1+i), (n0+n1+i),  ((dxy_0.row(5)).transpose()).rows(),
                                                                 ((dxy_0.row(5)).transpose()).cols()) =  (dxy_0.row(5)).transpose();
                       // step 4
                       DX_theta.block(nbs*(n0+n1+n2+i), (n0+n1+n2+i),  ((dxy_0.row(6)).transpose()).rows(),
                                                                       ((dxy_0.row(6)).transpose()).cols()) =  (dxy_0.row(6)).transpose();
                       DY_theta.block(nbs*(n0+n1+n2+i), (n0+n1+n2+i),  ((dxy_0.row(7)).transpose()).rows(),
                                                                       ((dxy_0.row(7)).transpose()).cols()) =  (dxy_0.row(7)).transpose();
                     }
                   else if (CpModelOption == 2)
                     {
                       for(int j=0; j<nu; j++)
                         {
                           // step 3
                           DX_theta.block(nbs*(n0+n1+i), (n0+n1+i),  ((dxy_0.row(4)).transpose()).rows(),
                                                                     ((dxy_0.row(4)).transpose()).cols()) =  (dxy_0.row(4)).transpose()*MpcModel->Pzu(n0+n1+i,j);
                           DY_theta.block(nbs*(n0+n1+i), (n0+n1+i),  ((dxy_0.row(5)).transpose()).rows(),
                                                                     ((dxy_0.row(5)).transpose()).cols()) =  (dxy_0.row(5)).transpose()*MpcModel->Pzu(n0+n1+i,j);
                           // step 4
                           DX_theta.block(nbs*(n0+n1+n2+i), (n0+n1+n2+i),  ((dxy_0.row(6)).transpose()).rows(),
                                                                           ((dxy_0.row(6)).transpose()).cols()) =  (dxy_0.row(6)).transpose()*MpcModel->Pzu(n0+n1+n2+i,j);
                           DY_theta.block(nbs*(n0+n1+n2+i), (n0+n1+n2+i),  ((dxy_0.row(7)).transpose()).rows(),
                                                                           ((dxy_0.row(7)).transpose()).cols()) =  (dxy_0.row(7)).transpose()*MpcModel->Pzu(n0+n1+n2+i,j);
                         }
                     }

                   // DX_k_x and DY_Uk_y
                   for (int j=0;j<nuk; j++)
                   {
                       // step 3
                       DX_Uk_x.block(nbs*(n0+n1+i), j,  ((dxy_0.row(4)).transpose()).rows(),
                                                        ((dxy_0.row(4)).transpose()).cols()) = (dxy_0.row(4)).transpose()*(SMx->GlobalFutureSelX)(n0+n1+i,j);
                       DY_Uk_y.block(nbs*(n0+n1+i), j,  ((dxy_0.row(5)).transpose()).rows(),
                                                        ((dxy_0.row(5)).transpose()).cols()) =  (dxy_0.row(5)).transpose()*(SMx->GlobalFutureSelY)(n0+n1+i,j);
                       // step 4
                       DX_Uk_x.block(nbs*(n0+n1+n2+i), j,   ((dxy_0.row(6)).transpose()).rows(),
                                                            ((dxy_0.row(6)).transpose()).cols()) =  (dxy_0.row(6)).transpose()*(SMx->GlobalFutureSelX)(n0+n1+n2+i,j);
                       DY_Uk_y.block(nbs*(n0+n1+n2+i), j,   ((dxy_0.row(7)).transpose()).rows(),
                                                            ((dxy_0.row(7)).transpose()).cols()) =  (dxy_0.row(7)).transpose()*(SMx->GlobalFutureSelY)(n0+n1+n2+i,j);
                   }
                   // DX_Szs_Ux0 and DX_Szs_Uy0
                   // step 3
                   DX_S_Ux0.segment(nbs*(n0+n1+i), ((dxy_0.row(4)).transpose()).rows()) =  (dxy_0.row(4)).transpose()*S_Ux0(n0+n1+i);
                   DY_S_Uy0.segment(nbs*(n0+n1+i), ((dxy_0.row(5)).transpose()).rows()) =  (dxy_0.row(5)).transpose()*S_Uy0(n0+n1+i);
                   // step 4
                   DX_S_Ux0.segment(nbs*(n0+n1+n2+i), ((dxy_0.row(6)).transpose()).rows()) =  (dxy_0.row(6)).transpose()*S_Ux0(n0+n1+n2+i);
                   DY_S_Uy0.segment(nbs*(n0+n1+n2+i), ((dxy_0.row(7)).transpose()).rows()) =  (dxy_0.row(7)).transpose()*S_Uy0(n0+n1+n2+i);
               }


               VectorXd B_theta;
               B_theta.resize(nbs*(n0+n1+n2+n2));
               B_theta.setZero(nbs*(n0+n1+n2+n2));


               for (int i=0; i<(n0+n1+n2+n2); i++)
               {
                   B_theta.segment(nbs*i, (EdgePositions).rows()) = EdgePositions;
               }

               // Overall Constraints Matrix
               MuAct.resize(nbs*(n0+n1+n2+n2), 2*nu + nuk);
               MuAct.setZero(nbs*(n0+n1+n2+n2), 2*nu + nuk);

               MuAct.block(0, 0, DX_theta.rows(), DX_theta.cols()) =  DX_theta;
               MuAct.block(0, nu, DY_theta.rows(), DY_theta.cols()) =  DY_theta;
               MuAct.block(0, 2*nu, (-DX_Uk_x-DY_Uk_y).rows(), (-DX_Uk_x-DY_Uk_y).cols()) =  -DX_Uk_x-DY_Uk_y;

               // Overall Constraints Vector
               BuAct.resize(nbs*(n0+n1+n2+n2));
               //BuAct.setZero(nbs*(n0+n1+2*n2));

               BuAct = B_theta + (DX_S_Ux0 + DY_S_Uy0);


//               cout << "B_theta is :\n"<< B_theta << endl;
//               cout << "MuAct is :\n"<< MuAct << endl;
//               cout << "(DX_S_Ux0 + DY_S_Uy0) is :\n"<< (DX_S_Ux0 + DY_S_Uy0) << endl;

   }

void CpConstraintsOutputZmp::FindReducedZmpConstraints12(MpcBased_CP_Model *MpcModel,
                                                            CP_SelectionMatrices *SMx,
                                                            Discrete_CP_LIP_Model *DMod,
                                                            CpStanceFootPose *CoPRef)
{
    int n0, n1, n2, nu, nbs, nuk;

        n0  = (MpcModel->SamplesPerStep)[0]; //nb of samples in Step1
        n1  = (MpcModel->SamplesPerStep)[1]; //nb of samples in Step2
        n2  = (MpcModel->SamplesPerStep)[2]; //nb of samples in Step3  4

        nu  = (MpcModel->PEu).cols(); // nb of columns of Pzu matrix
        nbs = EdgeNormals.cols();  // nb of vertices of support foot
        nuk = (SMx->GlobalFutureSelX).cols(); // nb of future steps

        // Stance Foot Rotation matrix over four steps
        MatrixXd Rot_theta;
        Rot_theta.resize(4,2);
        // current stance foot
        Rot_theta(0,0) =  cos((SMx->Theta_k1)(0));
        Rot_theta(0,1) = -sin((SMx->Theta_k1)(0));
        Rot_theta(1,0) =  sin((SMx->Theta_k1)(0));
        Rot_theta(1,1) =  cos((SMx->Theta_k1)(0));
        // first predicted step
        Rot_theta(2,0) =  cos((SMx->Theta_k1)(n0));
        Rot_theta(2,1) = -sin((SMx->Theta_k1)(n0));
        Rot_theta(3,0) =  sin((SMx->Theta_k1)(n0));
        Rot_theta(3,1) =  cos((SMx->Theta_k1)(n0));




        // normal to the support polygon (foot) vertices over the
        // prediction horizon (four footsteps)
        MatrixXd dxy_0;
        dxy_0 = Rot_theta * EdgeNormals;


        /* Product of Rotated X and Y components of normals to
         * support polygon vertices with EACH SAMPLE of the matrices
         * Pzu, Uk_x, Uk_y, and vector (Uc_k*Xc_k_h - Pzs*x_k_h)
         * (h=x,y respectively).
         *
         * X components of rotated normals over the four steps
         *    (dxy_0.row(1)) : current step (step1)
         *    (dxy_0.row(3)) : first future step (step2)
         *    (dxy_0.row(5)) : second future step (step3)
         *    (dxy_0.row(7)) : third future step (step4)
         *
         *
         * Y components of rotated normals over the four steps
         *    (dxy_0.row(2)) : current step (step1)
         *    (dxy_0.row(4)) : first future step (step2)
         *    (dxy_0.row(6)) : second future step (step3)
         *    (dxy_0.row(8)) : third future step (step4)
         *
         * ******************************************************** */

            MatrixXd DX_theta, DX_Uk_x,
                     DY_theta, DY_Uk_y;

            VectorXd DX_S_Ux0, DY_S_Uy0;

            // Initialisation
            //***************
            // combination matrix of the jerk and the constraints
            // Product with Matrix Pzu
            DX_theta.resize(nbs*(n0+n1),nu);
            DX_theta.setZero(nbs*(n0+n1),nu);

            DY_theta.resize(nbs*(n0+n1),nu);
            DY_theta.setZero(nbs*(n0+n1),nu);

            // combination matrix of relative footsteps to the constraints
            // Product with Matrix Uk_x and Uk_y
            DX_Uk_x.resize(nbs*(n0+n1),nuk);
            DX_Uk_x.setZero(nbs*(n0+n1),nuk);

            DY_Uk_y.resize(nbs*(n0+n1),nuk);
            DY_Uk_y.setZero(nbs*(n0+n1),nuk);

            // constraints of ZMP due to the state of the CoM
            // Product with Matrix
            DX_S_Ux0.resize(nbs*(n0+n1),1);
            DX_S_Ux0.setZero(nbs*(n0+n1),1);

            DY_S_Uy0.resize(nbs*(n0+n1),1);
            DY_S_Uy0.setZero(nbs*(n0+n1),1);

            // vector of Zmp constraints due to the state of the CoM and
            // the current stance foot position
            VectorXd S_Ux0, S_Uy0;


            if (CpModelOption == 0 || CpModelOption == 1)
              {
                S_Ux0 = SMx->GlobalCurrentSelX * CoPRef->CoPRefX; // Uckx_fcx = Uc_k_x*xk_fc_x;
                S_Uy0 = SMx->GlobalCurrentSelY * CoPRef->CoPRefY;
              }
            else if (CpModelOption == 2)
              {
                S_Ux0 = SMx->GlobalCurrentSelX * CoPRef->CoPRefX - MpcModel->Pzs * DMod->StatesX; // Uckx_fcx = Uc_k_x*xk_fc_x;
                S_Uy0 = SMx->GlobalCurrentSelY * CoPRef->CoPRefY - MpcModel->Pzs * DMod->StatesY;
              }

            // Computation
            //************

            MatrixXd dx_theta1, dx_theta2,
                     dy_theta1, dy_theta2;
            // step 1
            dx_theta1.resize(nbs*n0, n0);
            dx_theta1.setZero(nbs*n0, n0);

            dy_theta1.resize(nbs*n0, n0);
            dy_theta1.setZero(nbs*n0, n0);
            // step 2
            dx_theta2.resize(nbs*n1, n1);
            dx_theta2.setZero(nbs*n1, n1);

            dy_theta2.resize(nbs*n1, n1);
            dy_theta2.setZero(nbs*n1, n1);



            // step 1
             for (int i=0; i<n0; i++)
             {
                 dx_theta1.block(nbs*i, i, ((dxy_0.row(0)).transpose()).rows(),
                                 ((dxy_0.row(0)).transpose()).cols()) =  (dxy_0.row(0)).transpose();
                 dy_theta1.block(nbs*i, i, ((dxy_0.row(1)).transpose()).rows(),
                                 ((dxy_0.row(1)).transpose()).cols()) =  (dxy_0.row(1)).transpose();
             }

             // Step 2
             for (int i=0; i<n1; i++)
             {
                 dx_theta2.block(nbs*i, i,   ((dxy_0.row(2)).transpose()).rows(),
                                  ((dxy_0.row(2)).transpose()).cols()) =  (dxy_0.row(2)).transpose();
                 dy_theta2.block(nbs*i, i,   ((dxy_0.row(3)).transpose()).rows(),
                                 ((dxy_0.row(3)).transpose()).cols()) =  (dxy_0.row(3)).transpose();
             }

             // Grouping dx_theta1 and dx_theta2 in DX_theta

             if (CpModelOption == 0 || CpModelOption == 1)
               {
                 //step 1
                 DX_theta.block(0, 0, dx_theta1.rows(), dx_theta1.cols()) = dx_theta1;
                 // step 2
                 DX_theta.block(dx_theta1.rows(), dx_theta1.cols(), dx_theta2.rows(), dx_theta2.cols()) = dx_theta2;

                 // Grouping dy_theta1 and dy_theta2 in DY_theta
                 // step 1
                 DY_theta.block(0, 0, dy_theta1.rows(), dy_theta1.cols()) = dy_theta1;
                 // step 2
                 DY_theta.block(dy_theta1.rows(), dy_theta1.cols(), dy_theta2.rows(), dy_theta2.cols()) = dy_theta2;

               }
             else if (CpModelOption == 2)
               {
                 //step 1
                 DX_theta.block(0, 0, dx_theta1.rows(), nu) = dx_theta1 * (MpcModel->Pzu).block(0, 0, n0, nu);
                 // step 2
                 DX_theta.block(dx_theta1.rows(), 0, dx_theta2.rows(), nu) = dx_theta2 * (MpcModel->Pzu).block(n0, 0, n1, nu);

                 // Grouping dy_theta1 and dy_theta2 in DY_theta
                 // step 1
                 DY_theta.block(0, 0, dy_theta1.rows(), nu) = dy_theta1 * (MpcModel->Pzu).block(0, 0, n0, nu);
                 // step 2
                 DY_theta.block(dy_theta1.rows(), 0, dy_theta2.rows(), nu) = dy_theta2 * (MpcModel->Pzu).block(n0, 0, n1, nu);

               }


             // DX_Uk_x and DY_Uk_y
             // step 1
             DX_Uk_x.block(0,      0, nbs*n0, nuk) = dx_theta1 * (SMx->GlobalFutureSelX).block(0, 0, n0, nuk);
             // step 2
             DX_Uk_x.block(nbs*n0, 0, nbs*n1, nuk) = dx_theta2 * (SMx->GlobalFutureSelX).block(n0, 0, n1, nuk);
             // step 1
             DY_Uk_y.block(0,      0, nbs*n0, nuk) = dy_theta1 * (SMx->GlobalFutureSelY).block(0, 0, n0, nuk);
             // step 2
             DY_Uk_y.block(nbs*n0, 0, nbs*n1, nuk) = dy_theta2 * (SMx->GlobalFutureSelY).block(n0, 0, n1, nuk);


             // DX_Szs_Ux0 and DX_Szs_Uy0
             // step 1
             DX_S_Ux0.segment(0,      nbs*n0) = dx_theta1 * S_Ux0.segment(0, n0);
             // step 2
             DX_S_Ux0.segment(nbs*n0, nbs*n1) = dx_theta2 * S_Ux0.segment(n0,n1);


             // step 1
             DY_S_Uy0.segment(0,      nbs*n0) = dy_theta1 * S_Uy0.segment(0, n0);
             // step 2
             DY_S_Uy0.segment(nbs*n0, nbs*n1) = dy_theta2 * S_Uy0.segment(n0,n1);

//             cout << "S_Ux0 is : \n"<< S_Ux0 << endl;
//             cout << "S_Uy0 is : \n"<< S_Uy0 << endl;


            VectorXd B_theta;
            B_theta.resize(nbs*(n0+n1));
            B_theta.setZero(nbs*(n0+n1));


            for (int i=0; i<(n0+n1); i++)
            {
                B_theta.segment(nbs*i, (EdgePositions).rows()) = EdgePositions;
            }

            // Overall Constraints Matrix
            MuAct.resize(nbs*(n0+n1), 2*nu + nuk);
            MuAct.setZero(nbs*(n0+n1), 2*nu + nuk);

            MuAct.block(0, 0, DX_theta.rows(), DX_theta.cols()) =  DX_theta;
            MuAct.block(0, nu, DY_theta.rows(), DY_theta.cols()) =  DY_theta;
            MuAct.block(0, 2*nu, (-DX_Uk_x-DY_Uk_y).rows(), (-DX_Uk_x-DY_Uk_y).cols()) =  -(DX_Uk_x+DY_Uk_y);



            //cout << "MuAct is : \n"<< MuAct << endl;

            // Overall Constraints Vector
            BuAct.resize(nbs*(n0+n1));
            //BuAct.setZero(nbs*(n0+n1+2*n2));

            BuAct = B_theta + (DX_S_Ux0 + DY_S_Uy0);


}


void CpConstraintsOutputZmp::SetSupportEdgesLimits(VectorXd Edge_lim)
{
    EdgePositions = Edge_lim;
};

void CpConstraintsOutputZmp::SetSupportEdgesNormals(MatrixXd Edge_Nor)
{
    EdgeNormals = Edge_Nor;
}

