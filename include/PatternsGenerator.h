

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

#ifndef PatternsGenerator_H
#define PatternsGenerator_H

#include <string>
#include <iostream>
#include <fstream>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>

#include "InitBalWlkParameters.h"
#include "TemplateModels.h"
#include "MPCOptimizer.h"

#include <qpOASES.hpp>


using namespace std;
using namespace Eigen;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


// ====================================================================================



// ====================================================================================


class CpFootTrajectories
{    



        
        VectorXd PrevStanceXY; // Previous stance foot position
        double PrevStanceAng;     // Previuous stance foot orientation

        // for 3D plot using robot model
        double DelThetap1;      // Relative first future footstep orientation
        double DelTheta0n1;     // Relative previous footstep orientation
        VectorXd sxyn1;   // Relative previous footstep position
        VectorXd sxyp1;   // Relative first future footstep position
        double MaxFootHeight;   // Maximum swing foot height
 //       double StepPeriod;    // Step period
        double tm;                    // time

    public :

        // Output Arguments
        double XTrajLeftFoot;     // X trajectory of the left foot
        double YTrajLeftFoot;     // Y trajectory of the left foot
        double ZTrajLeftFoot;     // Z trajectory of the left foot
        double AngTrajLeftFoot;   // Angular trajectory of the left foot
        //
        double XTrajRightFoot;    // X trajectory of the left foot
        double YTrajRightFoot;    // Y trajectory of the left foot
        double ZTrajRightFoot;    // Z trajectory of the left foot
        double AngTrajRightFoot;  // Angular trajectory of the left foot

        int StanceIndicator[2];

    
        CpFootTrajectories(int SptFt[],
                           VectorXd RelCoPXYR,
                           CpStanceFootPose *CoPRef, 
                           CP_SelectionMatrices *SMx,
                                      double z_max);

//        void InitializeFootTrajectories(int SptFt[],
//                                         VectorXd RelCoPXYR,
//                                         CpStanceFootPose CoPref,
//                                        CP_SelectionMatrices SMx,
//                                                    double z_max);

                                           
        void ComputeFootTrajectories(       int SptFt[],
                                            VectorXd RelCoPXYR,
                                        CpStanceFootPose *CoPRef,
                                      CP_SelectionMatrices *SMx);
                                            

        void SetMaxFootHeight(double z_max);
};


// ====================================================================================



// ====================================================================================


class CpGaitTransformations
{
    

    
        VectorXd TrslBaseCoM;      // Translation vector from base to CoM

        public :

        MatrixXd TrsfLeftLegBase;     // left foot relative to the base frame
        MatrixXd TrsfRightLegBase;    // right foot rekative to the base frame
        MatrixXd TrsfBaseInWorld;     // base relative to the world frame
        MatrixXd RefT_icubB_icubF;    // Reference transformation form icub base to icub foot frame

        VectorXd RefTransl_Base_in_Foot; // icub base position expresssed in icub stance foot frame.

  
        CpGaitTransformations(Discrete_CP_LIP_Model *St,
                                           int SptFt[],
                               CpStanceFootPose *CoPref,
                              CpFootTrajectories *FtTraj,
                              VectorXd t_B_CoM);
        
        void ComputeGaitTransforms(Discrete_CP_LIP_Model *St,
                                                int SptFt[],
                                    CpStanceFootPose *CoPref,
                                   CpFootTrajectories *FtTraj);
                                 
        void SetTranslationBaseCoM(VectorXd t_B_CoM);
        
};


// ====================================================================================



// ====================================================================================
// class CpDesiredFeetTransformations
// {
    

    
//         VectorXd TrslBaseCoM;      // Translation vector from base to CoM

//         public :

//         MatrixXd DesTrsfRealStanceFootInHorBase;
//         MatrixXd CurTrsfRealStanceFootInHorBase;
//         MatrixXd ReferenceTrsfHorFootInHorBase;
//         MatrixXd TrsfCurBaseInWorldIMU;
                  
//         MatrixXd FwKRightFootInBase;
//         MatrixXd FwKLeftFootInBase;
        
//         Matrix3d CurRotRealStFootInHorBase;
//         Matrix3d RotRealStFootInHorStFoot;
        
//         MatrixXd ReferenceTrsfSwingFootInHorBase;
//         MatrixXd DesTrsfSwingFootInCurBase;

//         MatrixXd DesTrsfLeftFootInHorBase;
//         MatrixXd DesTrsfRightFootInHorBase;
        
//         // Methods
          
//         CpDesiredFeetTransformations();

//         ~CpDesiredFeetTransformations();
        
//         void getDesiredFeetInCorrectedBase(int SptFt[],
//                                            yarp::sig::Vector LLegPose,
//                                            yarp::sig::Vector RLegPose,
//                                            yarp::sig::Vector m_orientation_rpy,
//                                            CpGaitTransformations *GTj);

//         void SetImuTransformation(yarp::sig::Vector IMU_RollPitchYaw);

//         Vector3d getEulerAnglesXYZ_FixedFrame(Matrix3d CurRotRealStFootInHorBase);

//         Matrix3d getComposedOrientFixedFrame(Vector3d OrientXY_F_hF);
                                        
//     //MatrixXd getDesiredSwingFootInCurBase();
//     void SetImuTransformation();
                                 
//         void SetTranslationBaseCoM(VectorXd t_B_CoM);
        
// };

// ====================================================================================



// ====================================================================================




class CpReactiveWalkingController
{
        // initialisation of the optimal solution

            VectorXd RelCoPXY;
            VectorXd OrienCoP;

            VectorXd RelCoPXYR;
            VectorXd RelCoPXYR_n1;
            VectorXd OptimSolXY_n1;

            MatrixXd MconsXY;
            VectorXd BconsXY;


       public :

            Discrete_CP_LIP_Model           *DMod;
            MpcBased_CP_Model               *MpcModel;
            CP_SelectionMatrices            *SMx;
            CpStanceFootPose                *CoPref;
            VelocitiesSetPoints             *VeloRef;
            CpQMatrix                       *QMx;
            CpPVectorQP                     *PVec;
            CpConstraintsFootsteps          *CnstrFtStp;
            CpConstraintsOutputZmp          *CnstrZmp;
            CP_QPSolver_OASES               *QPxySolver;
            CP_QPSolver_OASES               *QPAngSolver;
            CpFootTrajectories              *FtTraj;
            CpGaitTransformations           *GaitTrsf;
            // CpDesiredFeetTransformations    *DesFtTrsf;

            int SwitchCounter;
            int CtrlCounter;

            int Tgbs;   // triggered before switching of DSP constraints  
            int Tgas;   // triggered after switching of DSP constraints 
            int DTds;   // nb of samples before or after the switch 

            // limits of the support polygon
            VectorXd    Spolygon_limits;
            //int_t nWSR;
            int nWSR;

            bool WaitForContact;

            // methods

            CpReactiveWalkingController();

            ~CpReactiveWalkingController();

            void InitializeCpBalWlkController(InitBalWlkParameters *Parameters);

            void UpdateCpBalWlkController(InitBalWlkParameters *Parameters, VectorXd RelativeVelo, int CycleCounter);

            void ReleaseCpBalWlkController();

};

// ===============================================================================================

// ===============================================================================================

class SyncRobotModelStanceFoot
{
  public:

    int stance_left;
    int stance_right;
   double minForceThrshld;
      int minTimeThrshld;  

    SyncRobotModelStanceFoot();
    ~SyncRobotModelStanceFoot();

    void InitStanceSync(double F_Threshld, int T_Threshld);

              void SyncStancePhase(           int CycleCounter,
                                   InitBalWlkParameters *Parameters,
                      CpReactiveWalkingController *CpBalWlkController,
                                         VectorXd l_foot_FT_vector,
                                         VectorXd r_foot_FT_vector);

};

#endif // PatternsGenerator_H
