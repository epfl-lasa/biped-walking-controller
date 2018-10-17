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



#include <iostream>
#include <cmath>
#include "TemplateModels.h"


/*
 * Discrete_CP_LIP_Model_H : This class encodes the discrete template model
 * used to model the dynamics of biped robot standing on one foot.
 *
 * The models are based on the 3D LIPM with finite sized foot.
 * 
*/

Discrete_CP_LIP_Model::Discrete_CP_LIP_Model(double SpTime, double CoMHeight, int CpOption)
{
    
    g = 9.80;

    Ts     = SpTime;
    zc     = CoMHeight;

    CpModelOption = CpOption;

    W  = sqrt(g/zc);
    a1  = exp(W*Ts);
    b1  = exp(-W*Ts);

    Mass = 30.0;

    AlpFrc = -1./(W*Mass);

    ZMP_X = 0.0;
    ZMP_Y = 0.0;

    switch (CpModelOption)
      {
        case 0:  // with states : [com cp]

          // State Transition matrix
          MxA.resize(2,2);
          MxA(0,0) = b1;
          MxA(0,1) = a1*(1.-pow(b1, 2.))/2.;
          MxA(1,0) = 0.;
          MxA(1,1) = a1;

          // State Control vector
          VeB.resize(2);
          VeB(0) = 1.-a1*(1.+ pow(b1, 2.))/2.;
          VeB(1) = 1.-a1;

          // Stace Measurement Vector for CoM
          VeC.resize(2);
          VeC(0) = 0.;
          VeC(1) = 1.;

          // Disturbance
          B_Dist.resize(2);
          B_Dist(0) = AlpFrc * a1*(1.+pow(b1, 2.))/2.;
          B_Dist(1) = AlpFrc * (a1-1);

          // States vectors
          StatesX.resize(2);  StatesX.setZero(2);
          StatesY.resize(2);  StatesY.setZero(2);

          StatesDistX.resize(2);  StatesDistX.setZero(2);
          StatesDistY.resize(2);  StatesDistY.setZero(2);
      

          break;

        case 1: //with states :[com com_dot]

         // State Transition matrix
           MxA.resize(2,2);
           MxA(0,0) = a1 *(1.+pow(b1, 2.))/2.;
           MxA(0,1) = a1/W *(1.-pow(b1, 2.))/2.;
           MxA(1,0) = a1*W *(1.-pow(b1, 2.))/2.;
           MxA(1,1) = a1 *(1.+pow(b1, 2.))/2.;

           // State Control vector
           VeB.resize(2);
           VeB(0) = 1.-a1*(1.+ pow(b1, 2.))/2.;
           VeB(1) = -a1*W *(1.-pow(b1, 2.))/2.;

           // Stace Measurement Vector for CoM
           VeC.resize(2);
           VeC(0) = 1.;
           VeC(1) = 1./W;

           // Disturbance
           B_Dist.resize(2);
           B_Dist(0) = AlpFrc * a1*(1.+pow(b1, 2.))/2.;
           B_Dist(1) = AlpFrc * (a1-1);

           // States vectors
           StatesX.resize(2);   StatesX.setZero(2);
           StatesY.resize(2);   StatesY.setZero(2);

           StatesDistX.resize(2);  StatesDistX.setZero(2);
           StatesDistY.resize(2);  StatesDistY.setZero(2);

          break;

        case 2:  //with states :[com com_dot com_ddot]

          // State Transition matrix
          MxA.resize(3,3);
          MxA.setIdentity(3,3);
          MxA(0,1) = Ts;
          MxA(0,2) = pow(Ts,2.)/2.;
          MxA(1,2) = Ts;

          // State Control vector
          VeB.resize(3);
          VeB(0) = pow(Ts,3.)/6.;
          VeB(1) = pow(Ts,2.)/2.;
          VeB(2) = Ts;

          // Stace Measurement Vector for stance foot orientation
          VeC.resize(3);
          VeC(0) = 1.;
          VeC(1) = 1./W;
          VeC(2) = 0.;

          // States vectors
          StatesX.resize(3);  StatesX.setZero(3);
          StatesY.resize(3);  StatesY.setZero(3);

          StatesDistX.resize(3);  StatesDistX.setZero(3);
          StatesDistY.resize(3);  StatesDistY.setZero(3);

        break;

      }


    // State Transition matrix
    MxA_R.resize(3,3);
    MxA_R.setIdentity(3,3);
    MxA_R(0,1) = Ts;
    MxA_R(0,2) = pow(Ts,2.)/2.;
    MxA_R(1,2) = Ts;

    // State Control vector
    VeB_R.resize(3);
    VeB_R(0) = pow(Ts,3.)/6.;
    VeB_R(1) = pow(Ts,2.)/2.;
    VeB_R(2) = Ts;

    // Stace Measurement Vector for stance foot orientation
    VeC_R.resize(3);
    VeC_R(0) = 1.;
    VeC_R(1) = 0.;
    VeC_R(2) = 0.;

    // Output Vector for the ZMP
    VeCz.resize(3);   // Stace Measurement Vector for ZMP
    VeCz(0) = 1.;
    VeCz(1) = 0.0;
    VeCz(2) = -zc/g;

    StatesR.resize(3);  StatesR.setZero(3);

    StatesDistR.resize(3);  StatesDistR.setZero(3);

}

Discrete_CP_LIP_Model::~Discrete_CP_LIP_Model()
{
//    delete MxA;
//    delete VeB;
//    delete VeC;

//    delete MxA_R;
//    delete VeB_R;
//    delete VeC_R;

//    delete StatesX;
//    delete StatesY;
//    delete StatesR;
//    delete OutputX;
//    delete OutputY;
//    delete OutputR;

//    delete CtrlUX;
//    delete CtrlUY;
//    delete CtrlUR;
}

void Discrete_CP_LIP_Model::Create_CP_LIP_Model(double SpTime, double CoMHeight)
{

       Ts     = SpTime;
       zc     = CoMHeight;

       W  = sqrt(g/zc);
       a1  = exp(W*Ts);
       b1  = exp(-W*Ts);

       AlpFrc = 1/(W*Mass);

       switch (CpModelOption)
         {
           case 0:  // with states : [com cp]

             // State Transition matrix
             MxA.resize(2,2);
             MxA(0,0) = b1;
             MxA(0,1) = a1*(1.-pow(b1, 2.))/2.;
             MxA(1,0) = 0.;
             MxA(1,1) = a1;

             // State Control vector
             VeB.resize(2);
             VeB(0) = 1.-a1*(1.+ pow(b1, 2.))/2.;
             VeB(1) = 1.-a1;

             // Stace Measurement Vector for CoM
             VeC.resize(2);
             VeC(0) = 0.;
             VeC(1) = 1.;

             // Disturbance
             B_Dist.resize(2);
             B_Dist(0) = AlpFrc/W * a1*(1.-pow(b1, 2.))/2.;
             B_Dist(1) = AlpFrc/W * (a1-1);

             // States vectors
             StatesX.resize(2);   StatesX.setZero(2);
             StatesY.resize(2);   StatesY.setZero(2);

             break;

           case 1: //with states :[com com_dot]

            // State Transition matrix
              MxA.resize(2,2);
              MxA(0,0) = a1 *(1.+pow(b1, 2.))/2.;
              MxA(0,1) = a1/W *(1.-pow(b1, 2.))/2.;
              MxA(1,0) = a1*W *(1.-pow(b1, 2.))/2.;
              MxA(1,1) = a1 *(1.+pow(b1, 2.))/2.;

              // State Control vector
              VeB.resize(2);
              VeB(0) = 1.-a1*(1.+ pow(b1, 2.))/2.;
              VeB(1) = -a1*W *(1.-pow(b1, 2.))/2.;

              // Stace Measurement Vector for CoM
              VeC.resize(2);
              VeC(0) = 1.;
              VeC(1) = 1./W;

              // Disturbance
              B_Dist.resize(2);
              B_Dist(0) = AlpFrc/W * a1*(1.-pow(b1, 2.))/2.;
              B_Dist(1) = AlpFrc/W * (a1-1);

              // States vectors
              StatesX.resize(2);  StatesX.setZero(2);
              StatesY.resize(2);  StatesY.setZero(2);

             break;

           case 2:  //with states :[com com_dot com_ddot]

             // State Transition matrix
             MxA.resize(3,3);
             MxA.setIdentity(3,3);
             MxA(0,1) = Ts;
             MxA(0,2) = pow(Ts,2.)/2.;
             MxA(1,2) = Ts;

             // State Control vector
             VeB.resize(3);
             VeB(0) = pow(Ts,3.)/6.;
             VeB(1) = pow(Ts,2.)/2.;
             VeB(2) = Ts;

             // Stace Measurement Vector for stance foot orientation
             VeC.resize(3);
             VeC(0) = 1.;
             VeC(1) = 1./W;
             VeC(2) = 0.;

             // States vectors
             StatesX.resize(3);   StatesX.setZero(3);
             StatesY.resize(3);   StatesY.setZero(3);

           break;

         }


       // State Transition matrix
       MxA_R.resize(3,3);
       MxA_R.setIdentity(3,3);
       MxA_R(0,1) = Ts;
       MxA_R(0,2) = pow(Ts,2.)/2.;
       MxA_R(1,2) = Ts;

       // State Control vector
       VeB_R.resize(3);
       VeB_R(0) = pow(Ts,3.)/6.;
       VeB_R(1) = pow(Ts,2.)/2.;
       VeB_R(2) = Ts;

       // Stace Measurement Vector for stance foot orientation
       VeC_R.resize(3);
       VeC_R(0) = 1.;
       VeC_R(1) = 0.;
       VeC_R(2) = 0.;

       StatesR.resize(3);   StatesR.setZero(3);

}

       // Update of the state by LMPC optimal solution

void Discrete_CP_LIP_Model::UpdateStates_CP_LIP_M(double uX, double uY, double uR)
{

      // Optimal control input from LMPC Solver
         CtrlUX = uX;
         CtrlUY = uY;
         CtrlUR = uR;

      // State update
         StatesX = MxA * StatesX + VeB * CtrlUX;
         StatesY = MxA * StatesY + VeB * CtrlUY;
         StatesR = MxA_R * StatesR + VeB_R * CtrlUR;

      // Output values
         OutputX = VeC   * StatesX; // X CoM
         OutputY = VeC   * StatesY; // Y CoM
         OutputR = VeC_R * StatesR; // Theta

         ZMP_X = CtrlUX;
         ZMP_Y = CtrlUY;

         RowVectorXd VeC_ZMP(3);
         VeC_ZMP(0) = 1.0;
         VeC_ZMP(1) = 0.0;
         VeC_ZMP(2) = - zc/g;


         if (CpModelOption == 2)
         {
            ZMP_X = VeC_ZMP * StatesX;
            ZMP_Y = VeC_ZMP * StatesY;
         }

}

  // set the height of the CoM
void Discrete_CP_LIP_Model::SetCoMHeight(double CoMHeight)
{
      zc = CoMHeight;
};




// *************************************************************************************
/*
 * MpcBased_CP_Model : This class creates the prediction model associated with 
 * the chosen template model.
 *
*/
// *************************************************************************************

// creating a function that returns the dynamic matrix
// corresponding to a given sampling time
    // Translation
MatrixXd MpcBased_CP_Model::A(double Ti, double zc)
    {

        double W = sqrt(9.8/zc);
        double ai =exp(W*Ti);
        double bi =exp(-W*Ti);

        MatrixXd AA;

        switch (CpModelOption)
          {
            case 0:

              AA.resize(2,2);
              AA(0,0) = bi;
              AA(0,1) = ai*(1.-pow(bi, 2.))/2.;
              AA(1,0) = 0.;
              AA(1,1) = ai;

            break;

            case 1:
              AA.resize(2,2);
              AA(0,0) = ai *(1.+pow(bi, 2.))/2.;
              AA(0,1) = ai/W *(1.-pow(bi, 2.))/2.;
              AA(1,0) = ai*W *(1.-pow(bi, 2.))/2.;
              AA(1,1) = ai *(1.+pow(bi, 2.))/2.;

            break;

            case 2:
              AA.resize(3,3);
              AA.setIdentity(3,3);
              AA(0,1) = Ti;
              AA(0,2) = pow(Ti,2.)/2.;
              AA(1,2) = Ti;

            break;
          }

        return AA;
    };

    // Rotation
    MatrixXd A_R(double Ti)
    {
        MatrixXd AA;
        AA.resize(3,3);
        AA.setIdentity(3,3);

        AA(0,1) = Ti;
        AA(0,2) = pow(Ti,2.)/2.;
        AA(1,2) = Ti;

        return AA;
    };
// creating a function that returns the control vector
// corresponding to a given sampling time
    // Translation
VectorXd MpcBased_CP_Model::B(double Ti, double zc)
   {
       double W  = sqrt(9.8/zc);
       double ai = exp(W*Ti);
       double bi = exp(-W*Ti);

      VectorXd BB;
       switch (CpModelOption)
         {
           case 0:
             BB.resize(2);
             BB(0) = 1.-ai*(1.+ pow(bi, 2.))/2.;
             BB(1) = 1.-ai;

           break;

           case 1:

             BB.resize(2);
             BB(0) = 1.-ai*(1.+ pow(bi, 2.))/2.;
             BB(1) = -ai*W *(1.-pow(bi, 2.))/2.;

           break;

           case 2:
             BB.resize(3);
             BB(0) = pow(Ti,3.)/6.;
             BB(1) = pow(Ti,2.)/2.;
             BB(2) = Ti;

           break;

         }
       return BB;
   };
   // Rotation
VectorXd B_R(double Ti)
{

       VectorXd BB(3);
       BB(0) = pow(Ti,3.)/6.;
       BB(1) = pow(Ti,2.)/2.;
       BB(2) = Ti;

       return BB;
};

void MpcBased_CP_Model::CreateContMpc_CP_Model(double SamplingTime, // Sampling time
                                      int RecHorizon, // Receding Horizon
                                      int CtrlHorizon, //Control Horizon
                                      double CoMHeight, // height of the CoM
                                      int CpOption)   // LIPM model type
   {

              g = 9.80;

              Tp   = SamplingTime;
              Ny   = RecHorizon;
              Nu   = CtrlHorizon;
              Zc   = CoMHeight;



              CpModelOption = CpOption;

              SamplesVector.resize(Ny);
              for (int i=0; i<Ny; i++)
              {
                  SamplesVector[i] = Tp;
              }


      // Initialization for memo allocation
              Pps.resize(Ny,2);     Pps.setZero(Ny,2);
              Pvs.resize(Ny,2);     Pvs.setZero(Ny,2);
              Pes.resize(Ny,2);     Pes.setZero(Ny,2);
              PEs.resize(Ny,2);     PEs.setZero(Ny,2);
              Pzs.resize(Ny,3);     Pzs.setZero(Ny,3);

              PpsR.resize(Ny,3);    PpsR.setZero(Ny,3);
              PvsR.resize(Ny,3);    PvsR.setZero(Ny,3);
              PzsR.resize(Ny,3);    PzsR.setZero(Ny,3);

              Ppu.resize(Ny,Ny);    Ppu.setZero(Ny,Ny);
              Pvu.resize(Ny,Ny);    Pvu.setZero(Ny,Ny);
              Peu.resize(Ny,Ny);    Peu.setZero(Ny,Ny);
              PEu.resize(Ny,Ny);    PEu.setZero(Ny,Ny);
              Pzu.resize(Ny,Ny);    Pzu.setZero(Ny,Ny);

              PpuR.resize(Ny,Ny);   PpuR.setZero(Ny,Ny);
              PvuR.resize(Ny,Ny);   PvuR.setZero(Ny,Ny);
              PzuR.resize(Ny,Ny);   PzuR.setZero(Ny,Ny);

              TheP.resize(Ny,Ny);   TheP.setZero(Ny,Ny);
              e1.resize(Ny);
              e1.setZero(Ny);
              e1(0) = 1;

              //
              RowVectorXd C, Cz;

              switch(CpModelOption)
                {
                  case 0:
                    C.resize(2);
                    C(0) = 0.;
                    C(1) = 1.;

                    break;
                  case 1:
                    C.resize(2);
                    C(0) = 0.;
                    C(1) = 1./sqrt(g/Zc);
                    break;
                  case 2:
                    C.resize(3);
                    C(0) = 1.;
                    C(1) = 1./sqrt(g/Zc);
                    C(2) = 0.;
                    break;

                }
              // Cz to compute the ZMP for the constraint matrix (Pzs, Pzu)
              Cz.resize(3);
              Cz(0) = 1.;
              Cz(1) = 0.;
              Cz(2) = -Zc/g;


              // Computation of the An and CAn-1 Matrices
              int p, q, s, mb, nb;           // (p,q) are size of Aa  s the nbr of row C.
                                             // mb, nb are size of Bb (control vector)
                  p  = A(Tp, Zc).rows(); //2; //(DMod.MxA).rows();
                  q  = A(Tp, Zc).cols(); //2; //(DMod.MxA).cols();
                  s  = C.rows();
                  mb = B(Tp, Zc).rows();
                  nb = B(Tp, Zc).cols();

              // Stace Measurement Vector for CP and CoM
              RowVectorXd  C_R;
              C_R.resize(3);
              C_R(0) = 1.;
              C_R(1) = 0.;
              C_R(2) = 0.;


              // Initializing of An and CAn-1 Matrices
              MatrixXd A_nT, C_nT, AB_nT, CAB_nT,
                       A_RnT,C_R_nT, AB_RnT, C_RAB_nT, Cz_nT, CzAB_nT;

              A_nT.resize(p*Ny,q);      A_nT.setZero(p*Ny,q);
              C_nT.resize(s*Ny,q);      C_nT.setZero(s*Ny,q);
              Cz_nT.resize(1*Ny,3);     Cz_nT.setZero(1*Ny,3);

              A_RnT.resize(3*Ny,3);     A_RnT.setZero(3*Ny,3);
              C_R_nT.resize(1*Ny,3);    C_R_nT.setZero(1*Ny,3);
              //C_R_nT(1:s*nt,1:q) = 0;

                  for (int i=0; i<Ny; i++)
                  {
                      if (i==0)
                      {
                          A_nT.block(p*i, 0, A(Tp, Zc).rows(), A(Tp, Zc).cols()) = A(Tp, Zc);
                          A_RnT.block(3*i, 0, A_R(Tp).rows(), A_R(Tp).cols())= A_R(Tp);
                          C_nT.block(s*i, 0, (C*A(Tp, Zc)).rows(), (C*A(Tp, Zc)).cols()) = C*A(Tp, Zc);
                          C_R_nT.block(1*i, 0, (C_R*A_R(Tp)).rows(), (C_R*A_R(Tp)).cols()) = C_R*A_R(Tp);

                          Cz_nT.block(1*i, 0, (Cz*A_R(Tp)).rows(), (Cz*A_R(Tp)).cols()) = Cz*A_R(Tp);


                      }
                      else
                      {
                          MatrixXd A_nT_1, A_RnT_1;
                          A_nT_1.resize(p,q);
                          A_nT_1.setZero(p,q);
                          A_RnT_1.resize(3,3);
                          A_RnT_1.setZero(3,3);

                          for (int k=0; k<p; k++)
                          {
                              for (int l=0; l<q; l++)
                              {
                                  A_nT_1(k,l) = A_nT(p*(i-1)+k,l);
                              }
                          }
                          for (int k=0; k<3; k++)
                          {
                              for (int l=0; l<3; l++)
                              {
                                  A_RnT_1(k,l) = A_RnT(3*(i-1)+k,l);
                              }
                          }

                          A_nT.block(p*i, 0, (A(Tp, Zc)*A_nT_1).rows(), (A(Tp, Zc)*A_nT_1).cols()) = A(Tp, Zc)*A_nT_1;
                          C_nT.block(s*i, 0, (C*A(Tp, Zc)*A_nT_1).rows(), (C*A(Tp, Zc)*A_nT_1).cols()) = C*A(Tp, Zc)*A_nT_1;
                          A_RnT.block(3*i, 0, (A_R(Tp)*A_RnT_1).rows(), (A_R(Tp)*A_RnT_1).cols()) = A_R(Tp)*A_RnT_1;
                          C_R_nT.block(1*i, 0, (C_R*A_R(Tp)*A_RnT_1).rows(), (C_R*A_R(Tp)*A_RnT_1).cols())= C_R*A_R(Tp)*A_RnT_1;

                          Cz_nT.block(1*i, 0, (Cz*A_R(Tp)*A_RnT_1).rows(), (Cz*A_R(Tp)*A_RnT_1).cols())= Cz*A_R(Tp)*A_RnT_1;


                      }
                  }

              // Computation of the An-1B and CAn-1B Matrices
              // Initializing of An-1B and CAn-1B Matrices
               AB_nT.resize(Ny*mb,Ny*nb);
               CAB_nT.resize(s*Ny,Ny*nb);
               AB_RnT.resize(Ny*3,Ny*1);
               C_RAB_nT.resize(1*Ny,Ny*1);
               CzAB_nT.resize(1*Ny,Ny*1);

               // for rotation
               //C_RAB_nT.resize(s*Ny,Ny*nb);

                  for (int i=0; i<Ny; i++)
                  {
                      for (int j=i; j<Ny; j++)
                      {
                          if (j==i)
                          {
                              AB_nT.block(mb*j, nb*i, (B(Tp, Zc)).rows(), (B(Tp, Zc)).cols()) = B(Tp, Zc);
                              //CAB_nT(s*j, nb*i, (C*B(Tp, Zc)).rows(), (C*B(Tp, Zc)).cols()) = C*B(Tp, Zc);
                              CAB_nT(s*j, nb*i) = C*B(Tp, Zc);

                              // for rotation
                              AB_RnT.block(3*j, 1*i, (B_R(Tp)).rows(), (B_R(Tp)).cols()) = B_R(Tp);
                              //C_RAB_nT(1*j, 1*i, (C_R*B_R(Tp)).rows(), (C_R*B_R(Tp)).cols()) = C_R*B_R(Tp);
                              C_RAB_nT(1*j, 1*i) = C_R*B_R(Tp);

                              CzAB_nT(1*j, 1*i) = Cz*B_R(Tp);

                          }
                          else
                          {
                              MatrixXd AB_nT_1, AB_RnT_1;
                              AB_nT_1.resize(mb,nb);
                              AB_nT_1.setZero(mb,nb);
                              AB_RnT_1.resize(3,1);
                              AB_RnT_1.setZero(3,1);

                              for (int k=0; k<mb; k++)
                              {
                                  for (int l=0; l<nb; l++)
                                  {
                                      AB_nT_1(k,l) = AB_nT(mb*(j-1)+k,nb*i+l);
                                  }
                              }
                              //AB_nT_1 = AB_nT.block(mb*(j-1), nb*(i), mb, nb);


                              for (int k=0; k<3; k++)
                              {
                                  for (int l=0; l<1; l++)
                                  {
                                      AB_RnT_1(k,l) = AB_RnT(3*(j-1)+k,1*i+l);
                                  }
                              }
                              //std::cout<<"AB_nT_1 is :\n"<< AB_nT_1 << std::endl;

                              AB_nT.block(mb*j, nb*i, (A(Tp, Zc)*AB_nT_1).rows(), (A(Tp, Zc)*AB_nT_1).cols()) = A(Tp, Zc)*AB_nT_1;
                              CAB_nT.block(s*j, nb*i, (C*A(Tp, Zc)*AB_nT_1).rows(), (C*A(Tp, Zc)*AB_nT_1).cols()) = C*A(Tp, Zc)*AB_nT_1;
                              // for rotation
                              AB_RnT.block(3*j, 1*i, (A_R(Tp)*AB_RnT_1).rows(), (A_R(Tp)*AB_RnT_1).cols()) = A_R(Tp)*AB_RnT_1;
                              C_RAB_nT.block(1*j, 1*i, (C_R*A_R(Tp)*AB_RnT_1).rows(), (C_R*A_R(Tp)*AB_RnT_1).cols()) = C_R*A_R(Tp)*AB_RnT_1;

                              CzAB_nT.block(1*j, 1*i, (Cz*A_R(Tp)*AB_RnT_1).rows(), (Cz*A_R(Tp)*AB_RnT_1).cols()) = Cz*A_R(Tp)*AB_RnT_1;

                          }
                          //std::cout<<"AB_nT is :\n"<< AB_nT << std::endl;
                          //std::cout<<"CAB_nT is :\n"<< CAB_nT << std::endl;
                      }
                  }



              // resizing the matrices
                  Pps.resize(Ny,q);     Pps.setZero(Ny,q);
                  Pvs.resize(Ny,q);     Pvs.setZero(Ny,q);
                  Pes.resize(Ny,q);     Pes.setZero(Ny,q);
                  PEs.resize(Ny,q);     PEs.setZero(Ny,q);

              // extracting values
              for (int i=0; i<Ny; i++)
              {

                  for (int j=0; j<q; j++)
                    {
                      switch (CpModelOption)
                        {
                          case 0:
                            // Position states matrix over Horizon Ny
                            Pps(i,j) = A_nT(p*i,j);

                            // Velocity states matrix over horizon Ny
                            Pes(i,j) = A_nT(p*i+1,j);

                            break;
                          case 1:

                            // Position states matrix over Horizon Ny
                            Pps(i,j) = A_nT(p*i,j);

                            // Velocity states matrix over horizon Ny
                            Pvs(i,j) = A_nT(p*i+1,j);

                            break;
                          case 2:

                            // Position states matrix over Horizon Ny
                            Pps(i,j) = A_nT(p*i,j);

                            // Velocity states matrix over horizon Ny
                            Pvs(i,j) = A_nT(p*i+1,j);
                            break;
                        }
                    }


                  // Angular Position states matrix over Horizon Ny
                      PpsR(i,0) = A_RnT(3*i,0);
                      PpsR(i,1) = A_RnT(3*i,1);
                      PpsR(i,2) = A_RnT(3*i,2);


                  // Angular Velocity states matrix over horizon Ny
                      PvsR(i,0) = A_RnT(3*i+1,0);
                      PvsR(i,1) = A_RnT(3*i+1,1);
                      PvsR(i,2) = A_RnT(3*i+1,2);

                  for (int j=0; j<Ny; j++)
                  {
                      switch (CpModelOption)
                        {
                        case 0:
                          // Psition control matrix over Horizon Ny
                          Ppu(i,j) = AB_nT(p*i,j);
                          // Velocity control matrix over Horizon Ny
                          Peu(i,j) = AB_nT(p*i+1,j);
                          break;
                        case 1:
                          // Psition control matrix over Horizon Ny
                          Ppu(i,j) = AB_nT(p*i,j);
                          // Velocity control matrix over Horizon Ny
                          Pvu(i,j) = AB_nT(p*i+1,j);
                          break;
                        case 2:
                          // Psition control matrix over Horizon Ny
                          Ppu(i,j) = AB_nT(p*i,j);
                          // Velocity control matrix over Horizon Ny
                          Pvu(i,j) = AB_nT(p*i+1,j);
                          break;

                        }

                      // Angular Psition control matrix over Horizon Ny
                      PpuR(i,j) = AB_RnT(3*i,j);
                      // Angular Velocity control matrix over Horizon Ny
                      PvuR(i,j) = AB_RnT(3*i+1,j);

                  }
              }



              // Capture point Output states matrix
              PEs = C_nT;
              // capture point Output control matrix
              PEu = CAB_nT;

              // ZMP Output states matrix
              Pzs = Cz_nT;
              // ZMP Output control matrix
              Pzu = CzAB_nT;

              // Rotational motion
              PzsR = PpsR;
              PzuR = PpuR;

              for (int i=0; i<Ny; i++)
              {
                   for (int j =0; j< Ny; j++)
                   {
                       if (j==i){ TheP(j,i) = 1.;}
                       else
                       {
                           if (j== i+1){ TheP(j,i) = -1.;}
                           else{ TheP(j,i) = 0.;}
                       }
                   }
               }
              // sub-state control matrix over Nu*T
              //-----------------------------------
   //            Ppu.resize(Ny,Nu,false);
   //            Pvu.resize(Ny,Nu,false);
   //            Pau.resize(Ny,Nu,false);
   //            Pzu.resize(Ny,Nu,false);
   //            PzuR.resize(Ny,Nu,false);

              // Matrix that computes the average velocity

                MatrixXd Avmx, Edm;
                Avmx.resize(Ny/2,Ny);
                Avmx.setZero(Ny/2,Ny);
                Edm.resize(Ny/2,Ny/2);
                Edm.setIdentity(Ny/2, Ny/2);

                Avmx.block(0,0, (-Edm).rows(), (-Edm).cols()) = -Edm;
                Avmx.block(0,(Ny/2), (Edm).rows(), (Edm).cols()) = Edm;

                Emx =  (1/(1*0.5*Ny*Tp))*Avmx;

   };



   void MpcBased_CP_Model::CreateSampMpc_CP_Model(Discrete_CP_LIP_Model *DMod, // Discrete LIPM Object
                                                                VectorXd SampVec, //Sample vector
                                                                int nbSampStp[], // Vector of nb of samples per steps
                                                                double DurStep[], // Vector of steps durations
                                                                int CpOption  )  // Type of the LIPM Model
   {

              g = 9.80;


              Tp   = DMod->Ts;
              Ny   = int(SampVec.rows());
              SamplesVector = SampVec;
              for (int i=0; i<4; i++){
                  DurationSteps[i]  = DurStep[i];
                  SamplesPerStep[i] = nbSampStp[i];
              }
              DurationSteps[4] = DurStep[4];

              Nu   = Ny;
              Zc   = DMod->zc;

              CpModelOption = CpOption;

              // Computation of the An and CAn-1 Matrices
              int p, q, s, mb, nb; // (p,q) are size of Aa  s the nbr of row C.
                                   // mb, nb are size of Bb (control vector)
                  p = (DMod->MxA).rows();
                  q = (DMod->MxA).cols();
                  s = (DMod->VeC).rows();
                  mb = (DMod->VeB).rows();
                  nb = (DMod->VeB).cols();

              // Initialization for memo allocation
                  Pps.resize(Ny,q);     Pps.setZero(Ny,q);
                  Pvs.resize(Ny,q);     Pvs.setZero(Ny,q);
                  Pes.resize(Ny,q);     Pes.setZero(Ny,q);
                  PEs.resize(Ny,q);     PEs.setZero(Ny,q);
                  Pzs.resize(Ny,3);     Pzs.setZero(Ny,3);

                  PpsR.resize(Ny,3);    PpsR.setZero(Ny,3);
                  PvsR.resize(Ny,3);    PvsR.setZero(Ny,3);
                  PzsR.resize(Ny,3);    PzsR.setZero(Ny,3);

                  Ppu.resize(Ny,Ny);    Ppu.setZero(Ny,Ny);
                  Pvu.resize(Ny,Ny);    Pvu.setZero(Ny,Ny);
                  Peu.resize(Ny,Ny);    Peu.setZero(Ny,Ny);
                  PEu.resize(Ny,Ny);    PEu.setZero(Ny,Ny);
                  Pzu.resize(Ny,Ny);    Pzu.setZero(Ny,Ny);


                  PpuR.resize(Ny,Ny);   PpuR.setZero(Ny,Ny);
                  PvuR.resize(Ny,Ny);   PvuR.setZero(Ny,Ny);
                  PzuR.resize(Ny,Ny);   PzuR.setZero(Ny,Ny);

                  TheP.resize(Ny,Ny);   TheP.setZero(Ny,Ny);
                  e1.resize(Ny);
                  e1.setZero(Ny);
                  e1(0) = 1.;





   //   RowVectorXd C, C_R;
   //   C = DMod.VeC;
   //   C_R = DMod.VeC_R;
      // Stace Measurement Vector for CP and CoM
      RowVectorXd C, C_R, Cz;
   //   C.resize(2);
   //   C(0) = 0.;
   //   C(1) = 1.;

      C = DMod->VeC;

      C_R.resize(3);
      C_R(0) = 1.;
      C_R(1) = 0.;
      C_R(2) = 0.;

      Cz.resize(3);
      Cz(0) = 1.;
      Cz(1) = 0.;
      Cz(2) = -Zc/g;


      // Initializing of An and CAn-1 Matrices
      MatrixXd A_nT, C_nT, AB_nT, CAB_nT,
               A_RnT,C_R_nT, AB_RnT, C_RAB_nT, Cz_nT, CzAB_nT;

      A_nT.resize(p*Ny,q);
      A_nT.setZero(p*Ny,q);
      C_nT.resize(s*Ny,q);
      C_nT.setZero(s*Ny,q);
      A_RnT.resize(3*Ny,3);
      A_RnT.setZero(3*Ny,3);
      C_R_nT.resize(1*Ny,3);
      C_R_nT.setZero(1*Ny,3);

      Cz_nT.resize(1*Ny,3);
      Cz_nT.setZero(1*Ny,3);

      //C_R_nT(1:s*nt,1:q) = 0;

          for (int i=0; i<Ny; i++)
          {
              if (i==0)
              {
                  A_nT.block(p*i, 0, (A(SampVec(i), Zc)).rows(), (A(SampVec(i), Zc)).cols()) = A(SampVec(i), Zc);
                  A_RnT.block(3*i, 0, (A_R(SampVec(i))).rows(), (A_R(SampVec(i))).cols()) = A_R(SampVec(i));
                  C_nT.block(i, 0, (C*A(SampVec(i), Zc)).rows(), (C*A(SampVec(i), Zc)).cols()) = C*A(SampVec(i), Zc);
                  C_R_nT.block(1*i, 0, (C_R*A_R(SampVec(i))).rows(), (C_R*A_R(SampVec(i))).cols()) = C_R*A_R(SampVec(i));

                  Cz_nT.block(1*i, 0, (Cz*A_R(SampVec(i))).rows(), (Cz*A_R(SampVec(i))).cols()) = Cz*A_R(SampVec(i));

              }
              else
              {
                  MatrixXd A_nT_1, A_RnT_1;
                  A_nT_1.resize(p,q);
                  A_nT_1.setZero(p,q);
                  A_RnT_1.resize(3,3);
                  A_RnT_1.setZero(3,3);

                  for (int k=0; k<p; k++)
                  {
                      for (int l=0; l<q; l++)
                      {
                          A_nT_1(k,l) = A_nT(p*(i-1)+k,l);
                      }
                  }
                  for (int k=0; k<3; k++)
                  {
                      for (int l=0; l<3; l++)
                      {
                          A_RnT_1(k,l) = A_RnT(3*(i-1)+k,l);
                      }
                  }

                  A_nT.block(p*i, 0, (A(SampVec(i), Zc)*A_nT_1).rows(), (A(SampVec(i), Zc)*A_nT_1).cols()) = A(SampVec(i), Zc)*A_nT_1;

                  C_nT.block(s*i, 0, (C*A(SampVec(i), Zc)*A_nT_1).rows(), (C*A(SampVec(i), Zc)*A_nT_1).cols()) = C*A(SampVec(i), Zc)*A_nT_1;

                  A_RnT.block(3*i, 0, (A_R(SampVec(i))*A_RnT_1).rows(), (A_R(SampVec(i))*A_RnT_1).cols()) = A_R(SampVec(i))*A_RnT_1;

                  C_R_nT.block(1*i, 0, (C_R*A_R(SampVec(i))*A_RnT_1).rows(), (C_R*A_R(SampVec(i))*A_RnT_1).cols()) = C_R*A_R(SampVec(i))*A_RnT_1;

                  Cz_nT.block(1*i, 0, (Cz*A_R(SampVec(i))*A_RnT_1).rows(), (Cz*A_R(SampVec(i))*A_RnT_1).cols()) = Cz*A_R(SampVec(i))*A_RnT_1;

              }
          }

   //       std::cout<<"A_nT is : \n" << A_nT << std::endl;
   //       std::cout<<"C_nT is : \n" << C_nT << std::endl;


      // Computation of the An-1B and CAn-1B Matrices
      // Initializing of An-1B and CAn-1B Matrices
       AB_nT.resize(Ny*mb,Ny*nb);
       AB_nT.setZero(Ny*mb,Ny*nb);
       CAB_nT.resize(s*Ny,Ny*nb);
       CAB_nT.setZero(s*Ny,Ny*nb);
       AB_RnT.resize(Ny*3,Ny*1);
       AB_RnT.setZero(Ny*3,Ny*1);
       C_RAB_nT.resize(1*Ny,Ny*1);
       C_RAB_nT.setZero(1*Ny,Ny*1);

       CzAB_nT.resize(1*Ny,Ny*1);
       CzAB_nT.setZero(1*Ny,Ny*1);
       // for rotation
       //C_RAB_nT.resize(s*Ny,Ny*nb);

          for (int i=0; i<Ny; i++)
          {
              for (int j=i; j<Ny; j++)
              {
                  if (j==i)
                  {
                      AB_nT.block(mb*j, nb*i, (B(SampVec(j), Zc)).rows(), (B(SampVec(j), Zc)).cols()) = B(SampVec(j), Zc);
                      CAB_nT(s*j,nb*i) = C*B(SampVec(j), Zc);
                      // for rotation
                      AB_RnT.block(3*j, 1*i, (B_R(SampVec(j))).rows(), (B_R(SampVec(j))).cols()) = B_R(SampVec(j));
                      C_RAB_nT(1*j, 1*i) = C_R*B_R(SampVec(j));

                      CzAB_nT(1*j, 1*i) = Cz*B_R(SampVec(j));

                  }
                  else
                  {
                      MatrixXd AB_nT_1, AB_RnT_1;
                      AB_nT_1.resize(mb,nb);
                      AB_nT_1.setZero(mb,nb);
                      AB_RnT_1.resize(3,1);
                      AB_RnT_1.setZero(3,1);


//                      for (int k=0; k<mb; k++)
//                      {
//                          for (int l=0; l<nb; l++)
//                          {
//                              AB_nT_1(k,l) = AB_nT(mb*(j-1)+k,nb*i+l);
//                          }
//                      }
                      AB_nT_1 = AB_nT.block(mb*(j-1), nb*(i), mb, nb);

                      for (int k=0; k<3; k++)
                      {
                          for (int l=0; l<1; l++)
                          {
                              AB_RnT_1(k,l) = AB_RnT(3*(j-1)+k,1*i+l);
                          }
                      }

                      AB_nT.block(mb*j, nb*i, (A(SampVec(j), Zc)*AB_nT_1).rows(), (A(SampVec(j), Zc)*AB_nT_1).cols()) = A(SampVec(j), Zc)*AB_nT_1;
                      CAB_nT.block(s*j, nb*i, (C*A(SampVec(j), Zc)*AB_nT_1).rows(), (C*A(SampVec(j), Zc)*AB_nT_1).cols()) = C*A(SampVec(j), Zc)*AB_nT_1;
                      // for rotation
                      AB_RnT.block(3*j, 1*i, (A_R(SampVec(j))*AB_RnT_1).rows(), (A_R(SampVec(j))*AB_RnT_1).cols()) = A_R(SampVec(j))*AB_RnT_1;
                      C_RAB_nT.block(1*j, 1*i, (C_R*A_R(SampVec(j))*AB_RnT_1).rows(), (C_R*A_R(SampVec(j))*AB_RnT_1).cols()) = C_R*A_R(SampVec(j))*AB_RnT_1;

                      CzAB_nT.block(1*j, 1*i, (Cz*A_R(SampVec(j))*AB_RnT_1).rows(), (Cz*A_R(SampVec(j))*AB_RnT_1).cols()) = Cz*A_R(SampVec(j))*AB_RnT_1;

                  }

              }
          }


//          std::cout<<"A_nT is :\n"<< A_nT << std::endl;
//          std::cout<<"C_nT is :\n"<< C_nT << std::endl;
//          std::cout<<"AB_nT is :\n"<< AB_nT << std::endl;
//          std::cout<<"CAB_nT is :\n"<< CAB_nT << std::endl;

          // resizing the matrices
              Pps.resize(Ny,q);     Pps.setZero(Ny,q);
              Pvs.resize(Ny,q);     Pvs.setZero(Ny,q);
              Pes.resize(Ny,q);     Pes.setZero(Ny,q);
              PEs.resize(Ny,q);     PEs.setZero(Ny,q);

          // extracting values
          for (int i=0; i<Ny; i++)
          {

              for (int j=0; j<q; j++)
                {
                  switch (CpModelOption)
                    {
                      case 0:
                        // Position states matrix over Horizon Ny
                        Pps(i,j) = A_nT(p*i,j);

                        // Velocity states matrix over horizon Ny
                        Pes(i,j) = A_nT(p*i+1,j);

                        break;
                      case 1:

                        // Position states matrix over Horizon Ny
                        Pps(i,j) = A_nT(p*i,j);

                        // Velocity states matrix over horizon Ny
                        Pvs(i,j) = A_nT(p*i+1,j);

                        break;
                      case 2:

                        // Position states matrix over Horizon Ny
                        Pps(i,j) = A_nT(p*i,j);

                        // Velocity states matrix over horizon Ny
                        Pvs(i,j) = A_nT(p*i+1,j);
                        break;
                    }
                }


              // Angular Position states matrix over Horizon Ny
                  PpsR(i,0) = A_RnT(3*i,0);
                  PpsR(i,1) = A_RnT(3*i,1);
                  PpsR(i,2) = A_RnT(3*i,2);


              // Angular Velocity states matrix over horizon Ny
                  PvsR(i,0) = A_RnT(3*i+1,0);
                  PvsR(i,1) = A_RnT(3*i+1,1);
                  PvsR(i,2) = A_RnT(3*i+1,2);

              for (int j=0; j<Ny; j++)
              {
                  switch (CpModelOption)
                    {
                    case 0:
                      // Psition control matrix over Horizon Ny
                      Ppu(i,j) = AB_nT(p*i,j);
                      // Velocity control matrix over Horizon Ny
                      Peu(i,j) = AB_nT(p*i+1,j);
                      break;
                    case 1:
                      // Psition control matrix over Horizon Ny
                      Ppu(i,j) = AB_nT(p*i,j);
                      // Velocity control matrix over Horizon Ny
                      Pvu(i,j) = AB_nT(p*i+1,j);
                      break;
                    case 2:
                      // Psition control matrix over Horizon Ny
                      Ppu(i,j) = AB_nT(p*i,j);
                      // Velocity control matrix over Horizon Ny
                      Pvu(i,j) = AB_nT(p*i+1,j);
                      break;

                    }

                  // Angular Psition control matrix over Horizon Ny
                  PpuR(i,j) = AB_RnT(3*i,j);
                  // Angular Velocity control matrix over Horizon Ny
                  PvuR(i,j) = AB_RnT(3*i+1,j);

              }
          }


      // Zmp Output states matrix
      PEs = C_nT;
      // Zmp Output control matrix
      PEu = CAB_nT;

      // ZMP Output states matrix
      Pzs = Cz_nT;
      // ZMP Output control matrix
      Pzu = CzAB_nT;

      // Rotational motion
      PzsR = PpsR;
      PzuR = PpuR;

      //std::cout<<"PEu is :\n"<< PzuR << std::endl;
      for (int i=0; i<Ny; i++)
      {
           for (int j =0; j< Ny; j++)
           {
               if (j==i){ TheP(j,i) = 1.;}
               else
               {
                   if (j== i+1){ TheP(j,i) = -1.;}
                   else{ TheP(j,i) = 0.;}
               }
           }
       }
       //std::cout<<"MpcModel Pps is : \n" << TheP << std::endl;

      // ----------------------------------------------
      // Emx matrix that compute average velocitie
      // ----------------------------------------------


   };

    // Matrix computing the average velocity from the predicted CoM
    // positions over Ny*T
   MatrixXd MpcBased_CP_Model::getEmx()
   {
       //MatrixXd EMX;
       Emx.resize(SamplesPerStep[0]+SamplesPerStep[1],Ny);
       Emx.setZero(SamplesPerStep[0]+SamplesPerStep[1],Ny);

       // Matrices E1 and E3 (related to Step 1 and 2)
       MatrixXd E1, E3;

       E1.resize(SamplesPerStep[0], SamplesPerStep[0]);
       E1.setIdentity(SamplesPerStep[0], SamplesPerStep[0]);

       E3.resize(SamplesPerStep[1], SamplesPerStep[1]);
       E3.setIdentity(SamplesPerStep[1], SamplesPerStep[1]);

       Emx.block(0, 0, E1.rows(), E1.cols()) = -(1./(2.*DurationSteps[0]))*E1;
       Emx.block(SamplesPerStep[0], SamplesPerStep[0], E3.rows(), E3.cols()) = -(1./(2.*DurationSteps[0]))*E3;

       VectorXd In, IndexVec(SamplesPerStep[0]);
       In = SamplesVector/Tp;
       IndexVec.setZero(SamplesPerStep[0]);
       for (int i=0; i<SamplesPerStep[0]; i++)
       {
           int sumv_i;
           sumv_i = 0;
           for(int j=0; j<i; j++)
           {
               sumv_i += (int)round(In(j));

           }

           IndexVec(i) = sumv_i;
       }




     // number of Ts samples per step3 sample time
     int nsf3 = (int) ((DurationSteps[0]/Tp)/SamplesPerStep[2]);



     VectorXd Ix0(nsf3), Ix1(nsf3);
     Ix0.setZero(nsf3); Ix1.setZero(nsf3);
     for (int i=0; i<nsf3; i++)
     {
         Ix0(i) = 1.-(i)*1/(double)(nsf3);
         Ix1(i) = (i)*1/(double)(nsf3);
     }

     // Matrix E2 (related to the 3rd step and defined in relation to step 1)
     MatrixXd E2;
     if (nsf3==1)
     {
         E2.resize(SamplesPerStep[0],SamplesPerStep[2]);
         E2.setZero(SamplesPerStep[0],SamplesPerStep[2]);
         for (int i=0; i<SamplesPerStep[0]; i++)
         {
             int index_i, s, j;
             index_i = IndexVec(i) +1; // +1 added to account for c++ indexing
             s = index_i - (int)((index_i-1)/nsf3)*nsf3;
             j = (int)((index_i-1)/nsf3)+1;

              E2(i,j-1) = Ix0(s-1); // -1 added to account for c++ indexing
          }
         //
         Emx.block(0, SamplesPerStep[0]+SamplesPerStep[1], E2.rows(), E2.cols()) = (1./(2.*DurationSteps[0]))*E2;
     }
     else
     {
         E2.resize(SamplesPerStep[0],SamplesPerStep[2]+1);
         E2.setZero(SamplesPerStep[0],SamplesPerStep[2]+1);
         for (int i=0; i<SamplesPerStep[0]; i++)
         {
             int index_i, s, j;
             index_i = IndexVec(i) +1; // +1 added to account for c++ indexing
             s = index_i - (int)((index_i-1)/nsf3)*nsf3;
             j = (int)((index_i-1)/nsf3)+1;

             E2(i,j-1) = Ix0(s-1); // -1 added to account for c++ indexing
             E2(i,j) = Ix1(s-1);
         }
         //
         Emx.block(0, SamplesPerStep[0]+SamplesPerStep[1]-1, E2.rows(), E2.cols()) = (1./(2.*DurationSteps[0]))*E2;
     }

     // nb of step2 samples per step4 sample time
     int nsf4 = (int)(SamplesPerStep[1]/SamplesPerStep[3]);
         Ix0.resize(nsf4);
         Ix0.setZero(nsf4);
         Ix1.resize(nsf4);
         Ix1.setZero(nsf4);


     for (int i=0; i<nsf4; i++)
     {
         Ix0(i) = 1-(i)*1/(double)(nsf4);
         Ix1(i) = (i)*1/(double)(nsf4);
     }

     // Matrix E4 (related to the 4th step and defined in relation to step 2)
     MatrixXd E4;
     if (nsf4 ==1)
     {
         E4.resize(SamplesPerStep[1],SamplesPerStep[2]);
         E4.setZero(SamplesPerStep[1],SamplesPerStep[2]);
         for (int i=0; i<SamplesPerStep[1]; i++)
         {
             int index_i, s, j;

             s = i+1 - (int)((i)/nsf4)*nsf4; // 1 added
             j = (int)((i)/nsf4)+1;

             E4(i,j-1) = Ix0(s-1); // -1 added to account for c++ indexing
         }
         //
         Emx.block(SamplesPerStep[0], Ny-SamplesPerStep[3], E4.rows(), E4.cols()) = (1./(2.*DurationSteps[0]))*E4;
     }
     else
     {
         E4.resize(SamplesPerStep[1],SamplesPerStep[2]+1);
         E4.setZero(SamplesPerStep[1],SamplesPerStep[2]+1);
         for (int i=0; i<SamplesPerStep[1]; i++)
         {
             int index_i, s, j;

             s = i+1 - (int)((i)/nsf4)*nsf4; // 1 added
             j = (int)((i)/nsf4)+1;

             E4(i,j-1) = Ix0(s-1); // -1 added to account for c++ indexing
             E4(i,j) = Ix1(s-1);
         }
         //
         Emx.block(SamplesPerStep[0], Ny-SamplesPerStep[3]-1, E4.rows(), E4.cols()) = (1./(2.*DurationSteps[0]))*E4;
     }

     return Emx; //2.2


   };

   void MpcBased_CP_Model::prefactorization()
   {
      PpuR_T_Emx_T = PpuR.transpose() * MpcBased_CP_Model::getEmx().transpose();
      TheP_T_e1    = TheP.transpose()*e1;
      Ppu_T_Emx_T  =Ppu.transpose() * MpcBased_CP_Model::getEmx().transpose();
      Emx_Pps      = MpcBased_CP_Model::getEmx() * Pps;

   }


   // Dynamic Filter for the CoM
   void MpcBased_CP_Model::CpDynamicFilter(Discrete_CP_LIP_Model *DMod,       // Discrete LIPM Object
                                                        VectorXd SampVec,     //Sample vector
                                                             int nbSampStp[], // array of nb of samples per steps
                                                          double DurStep[],   // array of steps durations
                                                             int CpOption)
   {
        

        // Computation of the An and CAn-1 Matrices
        int q; // (p,q) are size of Aa  s the nbr of row C.
                                   
            q = (DMod->MxA).cols();
        
        int ns1 = SamplesPerStep[0];

        // Initialization for memo allocation
        PEsDf.resize(ns1,q);      PEsDf.setZero(ns1,q);
        PEuDf.resize(ns1,ns1);    PEuDf.setZero(ns1,ns1);
        ThePDf.resize(ns1,ns1);   ThePDf.setZero(ns1,ns1);

        e1_Df.resize(ns1);        e1_Df.setZero(ns1);
        e1_Df(0) = 1.;


        PEsDf  = PEs.block(0, 0, ns1,q);
        PEuDf  = PEu.block(0, 0, ns1,ns1);
       
        for (int i=0; i<ns1; i++)
        {
             for (int j =0; j< ns1; j++)
             {
                 if (j==i){ ThePDf(j,i) = 1.;}
                 else
                 {
                     if (j== i+1){ ThePDf(j,i) = -1.;}
                     else{ ThePDf(j,i) = 0.;}
                 }
             }
         }


   }

// // ==============================================================================================================================================


// // ==============================================================================================================================================

CP_SelectionMatrices::CP_SelectionMatrices(VectorXd SampVec,
                                        double SamplingTime,
                                        int SamplesPerStep[],
                                        double DurationSteps[],
                                        double CoMHeight)
   {
       SamplesVector = SampVec;
       Tp = SamplingTime;
       for (int i=0; i<4; i++){
           SpS[i] = SamplesPerStep[i];
           DrS[i] = DurationSteps[i]; // DurationSteps[0]-[3]: Steps 1,2,3 & 4 durations.
       }
           DrS[4] = DurationSteps[4];     // DurationSteps[4]: double support duration

       RecH = SpS[0]+SpS[1]+SpS[2]+SpS[3];
       zc = CoMHeight;
       W = sqrt(9.81/zc);

       int tm   = 0; // tm for j in nk definition

       VectorXd Ones_3(3);
       Ones_3.setOnes(3);

       na = 0;
       na3=0;
       time_1 = 0;

       //

       //
       int ny,  // number of regularly spaced samples during step 1 period
       n1,  // actual number of samples over the prediction of step 1
       n2,  // actual number of samples over the prediction of step 2
       n3;  // actual number of samples over the prediction of step 3  4

       ny = (int)(round(DrS[0]/Tp));
       n1 = SpS[0];
       n2 = SpS[1];
       n3 = SpS[2];

       /* CYCLIC SELECTION MATRICES USED TO PREDETERMINE THE ZMP REFERENCE
        * Uc_k+1 and Uk+1 for X direction                            */
       // ********************************************************** //

       // Uc_k+1 : sampling vector of Current stance foot
       Uc_12.resize(2*ny + SpS[2]),
       Uc_23.resize(2*n2 + SpS[2]),
       Uc_34.resize(2*n3 + SpS[2]);
       // Creating a vector of ones and of dimension:
       VectorXd Ones_n1(ny), // (Tsp1/Tp)
                   Ones_n2(SpS[1]), // nb of samples in step 2
                   Ones_n3(SpS[2]); // nb of samples in step 3

       // assigning the value of 1 to all components
       Ones_n1.setOnes(ny);
       Ones_n2.setOnes(SpS[1]);
       Ones_n3.setOnes(SpS[2]);

       Uc_12.setZero(2*ny + SpS[2]);
       Uc_12.segment(0, Ones_n1.rows()) = Ones_n1;  // [1_ny]
       Uc_23.setZero(2*n2 + SpS[2]);               // [ 0  ]
       Uc_34.setZero(2*n3 + SpS[2]);               // [ 0  ]

       // Uf_k+1 : sampling vector of Future stance feet

       // 1st future step
       Uf_12.resize(2*ny +SpS[2], 3);
       Uf_12.setZero(2*ny +SpS[2], 3);
       // 2nd future step
       Uf_23.resize(2*SpS[1]+SpS[2], 3);
       Uf_23.setZero(2*SpS[1]+SpS[2], 3);
       // 3rd future step
       Uf_34.resize(2*SpS[2]+SpS[2], 3);
       Uf_34.setZero(2*SpS[2]+SpS[2], 3);

       Uf_12.block(ny, 0, Ones_n1.rows(), Ones_n1.cols()) = Ones_n1;        // [1_ny    0      0]
       Uf_23.block(0, 0, Ones_n2.rows(), Ones_n2.cols()) = Ones_n2;         // [1_n2    0        0]
       Uf_23.block(SpS[1], 1, Ones_n2.rows(), Ones_n2.cols()) = Ones_n2;    // [0      1_n2      0]
       Uf_34.block(0, 1, Ones_n3.rows(), Ones_n3.cols()) = Ones_n3;         // [0      1_n3      0]
       Uf_34.block(SpS[2], 2, Ones_n3.rows(), Ones_n3.cols()) = Ones_n3;    // [0       0     1_n3]
       Uf_34.block(2*SpS[2], 2, Ones_n3.rows(), Ones_n3.cols()) = Ones_n3;  // [0       0     1_n3]

           // Capture point reference trajectory
           int nds;  // nbre of samples in double support phase
               nds = (int)(round(DrS[4]/Tp));
           int Tgds; // last sample index of single support phase
               Tgds = (int)(round((DrS[0]-DrS[4])/Tp));
           // transition from single to double support: Interpoltion variables
           VectorXd U0ds(nds),
                    U1ds(nds);
               for (int i=0; i<nds; i++)
               {
                   // linear interpolation
                     //U0ds[i] = 1.-i/nds;
                     //U1ds[i] = i/nds;
                   // cubic polynomial interpolation
                     U0ds(i) = 1.-(3.*pow(i,2.)/pow(nds, 2.)-2.*pow(i,3.)/pow(nds, 3.));
                     U1ds(i) = 3.*pow(i,2.)/pow(nds, 2.)-2.*pow(i,3.)/pow(nds, 3.);
               }

           // sampling time of step 2 and step 3 over the prediction horizon
           double T2,
                  T3;
                  T2 = DrS[1]/SpS[1];
                  T3 = DrS[2]/SpS[2];

           // Matrix and vector to generate automatically the reference capture point

               double XwT1, XwT2, XwT3, XwT4;

                  XwT1 = (1-exp(-W*(DrS[0]-0.*Tp)));
                  XwT2 = (1-exp(-W*(DrS[1]-0.*Tp)));
                  XwT3 = (1-exp(-W*(DrS[2]-0.*Tp)));
                  XwT4 = (1-exp(-W*(DrS[3]-0.*Tp)));

           MatrixXd Fep1,      //Sub-matrix of Fep related to the first step with the sampling time Tp
                    Fep2T1,    //Sub-matrix of Fep related to the 2nd step with the sampling time Tp
                    Fep2,      //Sub-matrix of Fep related to the 2nd step with the sampling time T2
                    Fep3T2,    //Sub-matrix of Fep related to the 3rd step with the sampling time T2
                    Fep3,      //Sub-matrix of Fep related to the 3rd step with the sampling time T3
                    Fep4;      //Sub-matrix of Fep related to the 4th step with the sampling time T3

           VectorXd FeE1(ny),       //Sub-vector of FeE related to the 1st step with the sampling time Tp
                       FeE2T1(ny),     //Sub-vector of FeE related to the 2nd step with the sampling time Tp
                       FeE2(SpS[1]),   //Sub-vector of FeE related to the 2nd step with the sampling time T2
                       FeE3T2(SpS[1]), //Sub-vector of FeE related to the 3rd step with the sampling time T2
                       FeE3(SpS[2]),   //Sub-vector of FeE related to the 3rd step with the sampling time T3
                       FeE4(SpS[2]);   //Sub-vector of FeE related to the 4th step with the sampling time T3

           Fep1.resize(ny,4);
           Fep1.setZero(ny,4);
           Fep2T1.resize(ny,4);
           Fep2T1.setZero(ny,4);
           for (int i =0; i<ny; i++)
           {
             int j = i+0;

               if (i<=Tgds)
               {
                   Fep1(i,0) = exp(W*j*Tp)* XwT1 + (1-exp(W*j*Tp));
                   Fep1(i,1) = exp(W*j*Tp)* XwT2*exp(-W*DrS[0]);
                   Fep1(i,2) = exp(W*j*Tp)* XwT3*exp(-W*(DrS[1]+DrS[0]));
                   Fep1(i,3) = exp(W*j*Tp)* XwT4*exp(-W*(DrS[2]+DrS[1]+DrS[0]));
               }
               else
               {
                   Fep1(i,0) = exp(W*j*Tp)* XwT1*U0ds(i-Tgds) + (1-exp(W*i*Tp))*U0ds(i-Tgds);
                   Fep1(i,1) = exp(W*j*Tp)*(XwT1*U1ds(i-Tgds) + XwT2*exp(-W*DrS[0])) + (1-exp(W*j*Tp))*U1ds(i-Tgds);
                   Fep1(i,2) = exp(W*j*Tp)* XwT3*exp(-W*(DrS[1]+DrS[0]));
                   Fep1(i,3) = exp(W*j*Tp)* XwT4*exp(-W*(DrS[2]+DrS[1]+DrS[0]));
               }

               Fep2T1(i,0) = 0.;
               Fep2T1(i,1) = exp(W*j*Tp)* XwT2 + (1-exp(W*j*Tp));
               Fep2T1(i,2) = exp(W*j*Tp)* XwT3*exp(-W*DrS[1]);
               Fep2T1(i,3) = exp(W*j*Tp)* XwT4*exp(-W*(DrS[2]+DrS[1]));

               // Vector FeE
               FeE1(i)   = exp(W*j*Tp)*exp(-W*(DrS[3]+DrS[2]+DrS[1]+DrS[0]));

               FeE2T1(i) = exp(W*j*Tp)* exp(-W*(DrS[3]+DrS[2]+DrS[1]));

           }
          //
           Fep2.resize(SpS[1],4);
           Fep2.setZero(SpS[1],4);
           Fep3T2.resize(SpS[1],4);
           Fep3T2.setZero(SpS[1],4);
           for (int i=0; i<SpS[1]; i++)
           {
            int j = i+0;

               Fep2(i,0) = 0.;
               Fep2(i,1) = exp(W*j*T2)* XwT2 + (1-exp(W*j*T2));
               Fep2(i,2) = exp(W*j*T2)* XwT3*exp(-W*DrS[1]);
               Fep2(i,3) = exp(W*j*T2)* XwT4*exp(-W*(DrS[2]+DrS[1]));

               Fep3T2(i,0) = 0.;
               Fep3T2(i,1) = 0.;
               Fep3T2(i,2) = exp(W*j*T2)* XwT3 + (1-exp(W*j*T2));
               Fep3T2(i,3) = exp(W*j*T2)* XwT4*exp(-W*(DrS[2]));
               // Vector FeE
               FeE2(i)   = exp(W*j*T2)*exp(-W*(DrS[3]+DrS[2]+DrS[1]));

               FeE3T2(i) = exp(W*j*T2)*exp(-W*(DrS[3]+DrS[2]));

           }
           //
           Fep3.resize(SpS[2],4);
           Fep3.setZero(SpS[2],4);
           Fep4.resize(SpS[2],4);
           Fep4.setZero(SpS[2],4);
           for (int i=0; i<SpS[2]; i++)
           {
            int j = i+0;

               Fep3(i,0) = 0.;
               Fep3(i,1) = 0.;
               Fep3(i,2) = exp(W*j*T3)* XwT3+ (1-exp(W*j*T3));
               Fep3(i,3) = exp(W*j*T3)* XwT4*exp(-W*(DrS[2]));

               Fep4(i,0) = 0.;
               Fep4(i,1) = 0.;
               Fep4(i,2) = 0.;
               Fep4(i,3) = exp(W*j*T3)* XwT4 + (1-exp(W*j*T3));
               // Vector FeE
               FeE3(i) = exp(W*j*T3)*exp(-W*(DrS[3]+DrS[2]));

               FeE4(i) = exp(W*j*T3)*exp(-W*(DrS[3]));
           }
           // Assembling Fep Matrix and FeE vector
           Fep12.resize(2*ny,4);
           Fep12.setZero(2*ny,4);
           Fep12.block(0, 0, Fep1.rows(), Fep1.cols()) = Fep1;
           Fep12.block(ny, 0, Fep2T1.rows(), Fep2T1.cols()) = Fep2T1;

           Fep23.resize(2*SpS[1],4);
           Fep23.setZero(2*SpS[1],4);
           Fep23.block(0, 0, Fep2.rows(), Fep2.cols()) = Fep2;
           Fep23.block(SpS[1], 0, Fep3T2.rows(), Fep3T2.cols()) = Fep3T2;

           Fep34.resize(2*SpS[2]+SpS[2],4);
           Fep34.setZero(2*SpS[2]+SpS[2],4);
           Fep34.block(0, 0, Fep3.rows(), Fep3.cols()) = Fep3;
           Fep34.block(SpS[2], 0, Fep4.rows(), Fep4.cols()) = Fep4;


           FeE12.resize(2*ny);
           FeE12.setZero(2*ny);
           FeE12.segment(0, FeE1.rows()) = FeE1;
           FeE12.segment(ny, FeE2T1.rows()) = FeE2T1;

           FeE23.resize(2*SpS[1]);
           FeE23.setZero(2*SpS[1]);
           FeE23.segment(0, FeE2.rows()) = FeE2;
           FeE23.segment(SpS[1], FeE3T2.rows()) = FeE3T2;

           FeE34.resize(2*SpS[2]+SpS[2]);
           FeE34.setZero(2*SpS[2]+SpS[2]);
           FeE34.segment(0, FeE3.rows()) = FeE3;
           FeE34.segment(SpS[2], FeE4.rows()) = FeE4;
           FeE34.segment(2*SpS[2], Ones_n3.rows()) = Ones_n3;

       //
       // transition from single to double support: Interpoltion variables
           U0.resize(2*ny);
           Uf1.resize(2*ny);

           for (int i=0; i<ny-nds; i++)
           {
                U0(i)  = 1.;
                Uf1(i) = 0.;
           }
           for (int i=0; i<nds; i++)
           {
                U0(i+ny-nds)  = U0ds(i);
                Uf1(i+ny-nds) = U1ds(i);
           }
           for (int i=0; i<ny; i++)
           {
                U0(i+ny)  = 0.;
                Uf1(i+ny) = 1.;
           }


       // Intitialisation of class variable (tm = 0)
       CP_SelectionMatrices::CP_UpdateSelectionMx(0.,          // xk_fc_ang,
                                                   Ones_3 * 0., // OrienCoP,
                                                   tm);         // time_index;



   };

   void CP_SelectionMatrices::CP_UpdateSelectionMx(double xk_fc_ang,
                                                    VectorXd OrienCoP,
                                                    int time_index)
   {

           int tm;
           tm = time_index; // tm for j in nk definition

           int ny;  // number of regularly spaced samples during step 1 period
           ny = (int)(round(DrS[0]/Tp));


           // Creation of the cyclic mechanism
               int nk,  // cyclic variable with period ny*Tp
                  a12,  // cyclic variable with period (ny*Tp)*n2
                  a23;  // cyclic variable with period (ny*Tp)*n3

                  nk = fmod(tm,ny); // tm for j
                a12  = fmod(nk,round((DrS[0]/Tp)/SpS[1]));
                a23  = fmod(nk,round((DrS[0]/Tp)/SpS[2]));

                IndexSFt = nk;

                if (a23 ==0 && (tm!=time_1)) {
                    na3 = na3+1;
                }

                if (a12 ==0 && (tm!=time_1)) {
                    na = na+1;
                }

                if (nk==0) {
                        na=0;
                        na3 = 0;
                }

             // initialisation of na and na3
             if (tm == 0) {
                na = 0;
                na3 = 0;
             }

             time_1 = tm;

             // vector of position of step 1 samples over time
             VectorXd L(SpS[0]);
             VectorXd In, // vector of time between samples over horizon
               Index_S1(SpS[0]); // vector of samples position over Step 1

               In = SamplesVector*(1/Tp);
               Index_S1.setZero(SpS[0]);

            for (int i=0; i<SpS[0]; i++)
            {
                 int sumv_i;
                 sumv_i = 0;
                 for(int j=0; j<i; j++)
                 { sumv_i += round(In(j)); }

                   Index_S1(i) = sumv_i;
                   L(i) = Index_S1(i) + nk;
            }

               // Cyclic Selection Matrices in the local frame

               LocalCurrentSel.resize(RecH);
               LocalCurrentSel.setZero(RecH);

               LocalFutureSel.resize(RecH,3);
               LocalFutureSel.setZero(RecH,3);

               Fep_t.resize(RecH,4);
               Fep_t.setZero(RecH,4);
               FeE_t.resize(RecH);
               FeE_t.setZero(RecH);

               // Section related to step 1
               for (int i=0; i<SpS[0]; i++)
               {
                 LocalCurrentSel(i) = Uc_12(int(L(i)));

                 LocalFutureSel(i,0) = Uf_12(int(L(i)),0);
                 LocalFutureSel(i,1) = Uf_12(int(L(i)),1);
                 LocalFutureSel(i,2) = Uf_12(int(L(i)),2);

                 // Matrix and vector of reference Capture point
                 Fep_t(i,0) = Fep12(int(L(i)),0);
                 Fep_t(i,1) = Fep12(int(L(i)),1);
                 Fep_t(i,2) = Fep12(int(L(i)),2);
                 Fep_t(i,3) = Fep12(int(L(i)),3);

                 FeE_t(i) = FeE12(int(L(i)));

               }

               // Section related to step 2
               for (int i=0; i< SpS[1]; i++)
               {
                 LocalCurrentSel(SpS[0]+ i) = Uc_23(i+na);

                 LocalFutureSel(SpS[0]+ i,0) = Uf_23(i+na,0);
                 LocalFutureSel(SpS[0]+ i,1) = Uf_23(i+na,1);
                 LocalFutureSel(SpS[0]+ i,2) = Uf_23(i+na,2);

                 // Matrix and vector of reference Capture point
                 Fep_t(SpS[0]+ i,0) = Fep23(i+na,0);
                 Fep_t(SpS[0]+ i,1) = Fep23(i+na,1);
                 Fep_t(SpS[0]+ i,2) = Fep23(i+na,2);
                 Fep_t(SpS[0]+ i,3) = Fep23(i+na,3);

                 FeE_t(SpS[0]+ i) = FeE23(i+na);
               }
               // Sections related to step 3  4
               for (int i=0; i< 2*SpS[2]; i++)
               {
                 LocalCurrentSel(SpS[0]+SpS[1]+ i) = Uc_34(i+na3);

                 LocalFutureSel(SpS[0]+SpS[1]+ i,0) = Uf_34(i+na3,0);
                 LocalFutureSel(SpS[0]+SpS[1]+ i,1) = Uf_34(i+na3,1);
                 LocalFutureSel(SpS[0]+SpS[1]+ i,2) = Uf_34(i+na3,2);

                 // Matrix and vector of reference Capture point
                 Fep_t(SpS[0]+SpS[1]+ i,0) = Fep34(i+na3,0);
                 Fep_t(SpS[0]+SpS[1]+ i,1) = Fep34(i+na3,1);
                 Fep_t(SpS[0]+SpS[1]+ i,2) = Fep34(i+na3,2);
                 Fep_t(SpS[0]+SpS[1]+ i,3) = Fep34(i+na3,3);

                 FeE_t(SpS[0]+SpS[1]+ i) = FeE34(i+na3);
               }


           // Selection of the currrent and the future steps
               CP_LocalCurrentSel.resize(4);
               CP_LocalCurrentSel.setZero(4);
               CP_LocalCurrentSel(0) = 1.;

               CP_LocalFutureSel.resize(4,3);
               CP_LocalFutureSel.setZero(4,3);
               CP_LocalFutureSel(1,0) = 1.;
               CP_LocalFutureSel(2,1) = 1.;
               CP_LocalFutureSel(3,2) = 1.;

           // Test of Double support phase
               CP_LocalCurrentSel(0) = U0(nk);

               CP_LocalFutureSel(0,0) = Uf1(nk);


               // Determination Global cyclic selection matrices (world frame)

               Theta_k1 = LocalCurrentSel * xk_fc_ang +
                          LocalFutureSel *  OrienCoP;

               // Determination of footsteps orientation matrices with respect
               // the world frame
               // ************************************************************

               double cos_th1, sin_th1, // orientation of 1st future step
                      cos_th2, sin_th2, // orientation of 2nd future step
                      cos_th3, sin_th3; // orientation of 3rd future step

               cos_th1 = cos(Theta_k1(SpS[0]));
               sin_th1 = sin(Theta_k1(SpS[0]));
               //
               cos_th2 = cos(Theta_k1(SpS[0]+SpS[1]));
               sin_th2 = sin(Theta_k1(SpS[0]+SpS[1]));
               //
               cos_th3 = cos(Theta_k1(SpS[0]+SpS[1]+SpS[2]));
               sin_th3 = sin(Theta_k1(SpS[0]+SpS[1]+SpS[2]));

               // Rotatiom matrices

               RotMx.resize(3,6);
               RotMx.setZero(3,6);
               RotMy.resize(3,6);
               RotMy.setZero(3,6);

               // Rx matrix

               RotMx(0,0) =  cos_th1;  RotMx(1,0) =  cos_th1; RotMx(2,0) =  cos_th1;
               RotMx(0,1) = -sin_th1;  RotMx(1,1) = -sin_th1; RotMx(2,1) = -sin_th1;
               RotMx(0,2) =     0.;  RotMx(1,2) =  cos_th2; RotMx(2,2) =  cos_th2;
               RotMx(0,3) =       0.;  RotMx(1,3) = -sin_th2; RotMx(2,3) = -sin_th2;
               RotMx(0,4) =       0.;  RotMx(1,4) =        0.; RotMx(2,4) =  cos_th3;
               RotMx(0,5) =       0.;  RotMx(1,5) =        0.; RotMx(2,5) = -sin_th3;


               // Ry matrix

               RotMy(0,0) =  sin_th1;  RotMy(1,0) =  sin_th1; RotMy(2,0) =  sin_th1;
               RotMy(0,1) =  cos_th1;  RotMy(1,1) =  cos_th1; RotMy(2,1) =  cos_th1;
               RotMy(0,2) =     0.;  RotMy(1,2) =  sin_th2; RotMy(2,2) =  sin_th2;
               RotMy(0,3) =       0.;  RotMy(1,3) =  cos_th2; RotMy(2,3) =  cos_th2;
               RotMy(0,4) =       0.;  RotMy(1,4) =        0.; RotMy(2,4) =  sin_th3;
               RotMy(0,5) =       0.;  RotMy(1,5) =        0.; RotMy(2,5) =  cos_th3;

               // Global orientation matrices wrt. the world frame
               VectorXd Ones_3(3);
               Ones_3.setOnes(3);
               // New Uc_k_x and Uk_x
               GlobalCurrentSelX = LocalCurrentSel + LocalFutureSel * Ones_3;
               GlobalFutureSelX  = LocalFutureSel * RotMx;

               // New Uc_k_y and Uk_y
               GlobalCurrentSelY  = LocalCurrentSel + LocalFutureSel * Ones_3;
               GlobalFutureSelY   = LocalFutureSel * RotMy;

               // New Vcx and Vfx
               CP_GlobalCurrentSelX = CP_LocalCurrentSel + CP_LocalFutureSel * Ones_3; // Vcnx = Vc + Vf.1
               CP_GlobalFutureSelX  = CP_LocalFutureSel * RotMx;   // Vfnx = Vf * Rx

               // New Vcy and Vfy
               CP_GlobalCurrentSelY  = CP_LocalCurrentSel + CP_LocalFutureSel * Ones_3;
               CP_GlobalFutureSelY   = CP_LocalFutureSel * RotMy;

               // Rotations matrix of the end of step capture point

               // x
               RotX3 = RotMx.row(2);  //  third row

               RotX33.resize(2);
               RotX33(0) = RotMx(2,4);
               RotX33(1) = RotMx(2,5);
               // y
               RotY3 = RotMy.row(2);  // third row

               RotY33.resize(2);
               RotY33(0) = RotMy(2,4);
               RotY33(1) = RotMy(2,5);

               // Global CP selection Matrices
               // X
               GlobalCurrent_Fep_t_X = Fep_t * CP_GlobalCurrentSelX;  // (Fep_tx*Vcnx)
               GlobalFuture_Fep_t_X  = Fep_t * CP_GlobalFutureSelX + FeE_t * RotX3; // (Fep_tx*Vfnx + FeE_t*Rx3)

               GlobalFuture_FeE_t_X = FeE_t * RotX33;  // FeE_t*Rx33
               // Y
               GlobalCurrent_Fep_t_Y = Fep_t * CP_GlobalCurrentSelY;
               GlobalFuture_Fep_t_Y  = Fep_t * CP_GlobalFutureSelY + FeE_t * RotY3;

               GlobalFuture_FeE_t_Y = FeE_t * RotY33;

   }


// =================================================================================================================


// =================================================================================================================

CpStanceFootPose::CpStanceFootPose(VectorXd IniCoPrefPose)
{
           /* IniCoPrefPose is a vector of dimension 5
            * IniCoPrefPose[0] : Initial X position of stance foot
            * IniCoPrefPose[1] : Initial Y position of stance foot
            * IniCoPrefPose[2] : Initial Orientation of stance foot
            * IniCoPrefPose[3] : Initial X position of 1st future stf
            * IniCoPrefPose[4] : Initial Y position of 1st future stf*/

           CoPRefX = IniCoPrefPose(0);
           CoPRefY = IniCoPrefPose(1);
           CoPRefR = IniCoPrefPose(2);

           xi_0.resize(3);
           xi_0.setZero(3);

           yi_0.resize(3);
           yi_0.setZero(3);

           xi_0(0) = IniCoPrefPose(3);
           yi_0(0) = IniCoPrefPose(4); // e.g -0.05 Nao

           xk_fc_xy_n1.resize(2);
           xk_fc_xy_n1(0) = 0.;
           xk_fc_xy_n1(1) = 0.;

           xk_fc_ang_n1 = 0.;

           xCP_ref = CoPRefX;
           yCP_ref = CoPRefY;

}

void CpStanceFootPose::ReinitializeFootPose(VectorXd IniCoPrefPose)

{
           /* IniCoPrefPose is a vector of dimension 5
            * IniCoPrefPose[0] : Initial X position of stance foot
            * IniCoPrefPose[1] : Initial Y position of stance foot
            * IniCoPrefPose[2] : Initial Orientation of stance foot
            * IniCoPrefPose[3] : Initial X position of 1st future stf
            * IniCoPrefPose[4] : Initial Y position of 1st future stf*/

           CoPRefX = IniCoPrefPose(0);
           CoPRefY = IniCoPrefPose(1);
           CoPRefR = IniCoPrefPose(2);

           xi_0(0) = IniCoPrefPose(3);
           yi_0(0) = IniCoPrefPose(4); // -0.05 Nao

           xk_fc_xy_n1(0) = 0.;
           xk_fc_xy_n1(1) = 0.;

           xk_fc_ang_n1 = 0.;

           // xCP_ref = CoPRefX;
           // yCP_ref = CoPRefY;


}

void CpStanceFootPose::UpdateStanceFootPose(CP_SelectionMatrices *SMx,
                                           VectorXd RelCoPXYR,     // RelCoPXYR[0]-[5]: RelCoPXY;
                                           VectorXd RelCoPXYR_n1)  // RelCoPXYR[6]-[8]: OrienCoP;
                                                                      // RelCoPXYR_n1[0]-[1]: RelCoPXY_n1;
                                                                      // RelCoPXYR_n1[2]    : OrienCoP_n1;
{
           VectorXd RelCoPXY(6), RelEoSXY(2), OrienCoP(3);
               for (int i=0; i<6; i++)
               {
                   RelCoPXY(i) = RelCoPXYR(i);
               }
               //
               RelEoSXY(0) = RelCoPXYR(6);
               RelEoSXY(1) = RelCoPXYR(7);
               //
               OrienCoP(0) = RelCoPXYR(8);
               OrienCoP(1) = RelCoPXYR(9);
               OrienCoP(2) = RelCoPXYR(10);


           // cyclic variable indicating the change of stance foot
           //xk_fc_xy_n1.resize(2);

           if (SMx->IndexSFt == 0)
           {
               // relative position and angle of the previous step
               xk_fc_xy_n1(0)  = -RelCoPXYR_n1(0);  //x
               xk_fc_xy_n1(1)  = -RelCoPXYR_n1(1);  //y

               xk_fc_ang_n1    =  CoPRefR; //RelCoPXYR_n1[2];  // Rotation (theta)
               //
               // update of the current stance foot wrt. the world frame
               CoPRefX = xi_0(0);
               CoPRefY = yi_0(0);
               CoPRefR = OrienCoP(0);

           }

               // computation of the predicted steps wrt. the world frame
               VectorXd Ones_3(3);
               Ones_3.setOnes(3);

               xi_0 = Ones_3* CoPRefX + SMx->RotMx * RelCoPXY;

               yi_0 = Ones_3* CoPRefY + SMx->RotMy * RelCoPXY;


               // reference capture point references
               VectorXd zx_ref = SMx->CP_GlobalCurrentSelX * CoPRefX + SMx->CP_GlobalFutureSelX * RelCoPXY;
               VectorXd zy_ref = SMx->CP_GlobalCurrentSelY * CoPRefY + SMx->CP_GlobalFutureSelY * RelCoPXY;


               xCP_ref = (SMx->FeE_t * RelEoSXY(0) + SMx->Fep_t * zx_ref)(0);

               yCP_ref = (SMx->FeE_t * RelEoSXY(1) + SMx->Fep_t * zy_ref)(0);

};


// // ====================================================================================================================



// // ====================================================================================================================
void VelocitiesSetPoints::WorldVelocitiesSetPts(CP_SelectionMatrices *SMx, // Selection Matrices
                                                            VectorXd R_Velo) // Relative velocity vector (v_x, v_y, w_z)
{
   //
   // Creating a vector of ones and of dimension Ny (rec. horizon)
   VectorXd Ones_Ny((SMx->Theta_k1).rows());
   Ones_Ny.setOnes((SMx->Theta_k1).rows());

   // R_Velo[0]*Ones_Ny : v_x velocity over the predicted horizon
   // R_Velo[1]*Ones_Ny : v_y velocity over the predicted horizon
   // R_Velo[2]*Ones_Ny : w_z velocity over the predicted horizon
   //

   // COMPUTATION OF RELATIVE VELOCITIES
   //
   MatrixXd Cos_theta, //
            Sin_theta; //
   VectorXd c_theta((SMx->Theta_k1).rows()), //
            s_theta((SMx->Theta_k1).rows()); //

   for (unsigned int i=0; i<(SMx->Theta_k1).rows(); i++)
   {
       c_theta(i) = cos((SMx->Theta_k1)(i));
       s_theta(i) = sin((SMx->Theta_k1)(i));
   }

   // Creating diagonal matrices
   Cos_theta = c_theta.asDiagonal();
   Sin_theta = s_theta.asDiagonal();
               //
   // X velocity Velocity wrt. the world frame
   //
   VxAbs = Cos_theta*R_Velo(0)*Ones_Ny - Sin_theta*R_Velo(1)*Ones_Ny;

   // Y velocity Velocity wrt. the world frame
   VyAbs = Sin_theta*R_Velo(0)*Ones_Ny + Cos_theta*R_Velo(1)*Ones_Ny;

   // Angular velocity about the vertical axis
   WzAbs = R_Velo(2)*Ones_Ny;

}