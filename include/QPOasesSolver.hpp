#ifndef CP_QPSolver_OASES_H
#define CP_QPSolver_OASES_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/LU>

#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

using namespace std;
using namespace Eigen;

// QPOasesSolver.hpp

class QPOasesSolver
{
	public:

		  SQProblem OasesSolver;

		  //int_t nbWorkSetRecal;
           int nbWorkSetRecal;

           // Solving the QP problem

           void InitialiseQPSol(MatrixXd H,
                                VectorXd f,
                                MatrixXd A_cons,
                                VectorXd b)
           {
		       //intitialisation
		       
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

		           nbWorkSetRecal = 400;  //200

		           OasesSolver.init(HMx,gVc,AMx,0,0,0,ubA, nbWorkSetRecal, 0);

		           OasesSolver.getPrimalSolution(xOpt);


			};


           VectorXd qpOasesSolver (MatrixXd H,
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


            void setnbWorkingSetRecalculation(int nWSR)
            {
			    nbWorkSetRecal = nWSR;
		    };

            // void CP_QPSolution_Oases(MatrixXd Q,         // Hessian matrix
				        //           	VectorXd p_k,       // gradient vector
				        //           	MatrixXd M,         // Matrix of constraints
				        //           	VectorXd gam);      // vector of constraints

            void setSolverOptions(Options optionToSet)
            {
     			OasesSolver.setOptions(optionToSet);
		    };

};
#endif  // QPOasesSolver

