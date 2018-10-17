#ifndef CpMath_Utilities_H
#define CpMath_Utilities_H

#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>


#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

//#include "Discrete_CP_LIP_Model.h"
//#include "MpcBased_CP_Model.h"

#include "TemplateModels.h"

using namespace std;
using namespace Eigen;

using namespace yarp::sig;
using namespace yarp::math;


//

// =====================================================================================================================
// classes

class Transformations
{
   public:

      Transformations() {}
      ~Transformations() {}

      void Init(){}


      Eigen::MatrixXd yarpPoseTransform2EigenHmatrix(Vector yarpPoseVect)
      {
          Eigen::MatrixXd H_out(4,4);
          H_out.setIdentity(4,4);

          H_out(0,3) = yarpPoseVect[0];
          H_out(1,3) = yarpPoseVect[1];
          H_out(2,3) = yarpPoseVect[2];

          yarp::sig::Matrix RotMx;
          RotMx = yarp::math::axis2dcm(yarpPoseVect.subVector(3,6));
          // extracting data from a Eigen matrix to a yarp matrix
          for (int row=0; row<3; row++)
          {
              for (int col=0; col<3; col++)
              {
                  H_out(row, col) = RotMx(row,col);
              }
          }

          return H_out;

      }


      Eigen::MatrixXd AxisAngle2RotationMx(Eigen::VectorXd Pose_V)
      {
          //
          Eigen::MatrixXd RotationMx(3,3);
          //
          RotationMx(0,0) = cos(Pose_V(3)) + Pose_V(0)*Pose_V(0)*(1.0  - cos(Pose_V(3)));
          RotationMx(1,0) = Pose_V(1)*Pose_V(0)*(1.0 - cos(Pose_V(3))) + Pose_V(2)*sin(Pose_V(3));
          RotationMx(2,0) = Pose_V(2)*Pose_V(0)*(1.0 - cos(Pose_V(3))) - Pose_V(1)*sin(Pose_V(3));

          RotationMx(0,1) = Pose_V(0)*Pose_V(1)*(1.0 - cos(Pose_V(3))) - Pose_V(2)*sin(Pose_V(3));
          RotationMx(1,1) = cos(Pose_V(3)) + Pose_V(1)*Pose_V(1)*(1.0  - cos(Pose_V(3)));
          RotationMx(2,1) = Pose_V(2)*Pose_V(1)*(1.0 - cos(Pose_V(3))) + Pose_V(0)*sin(Pose_V(3));

          RotationMx(0,2) = Pose_V(0)*Pose_V(2)*(1.0 - cos(Pose_V(3))) + Pose_V(1)*sin(Pose_V(3));
          RotationMx(1,2) = Pose_V(1)*Pose_V(2)*(1.0 - cos(Pose_V(3))) - Pose_V(0)*sin(Pose_V(3));
          RotationMx(2,2) = cos(Pose_V(3)) + Pose_V(2)*Pose_V(2)*(1.0  - cos(Pose_V(3)));

          return RotationMx;

      }

      //
      Eigen::VectorXd rotationMx2axisangle(Eigen::Matrix3d Rot_mx)
      {
          //
          Eigen::VectorXd axis_angle(4);
          double angle;
          //
          // computation of the angle
          angle = acos((Rot_mx.trace()-1.)/2.);

        // computation of axis
        //
        axis_angle(0) = Rot_mx(2,1)-Rot_mx(1,2);
        axis_angle(1) = Rot_mx(0,2)-Rot_mx(2,0);
        axis_angle(2) = Rot_mx(1,0)-Rot_mx(0,1);

        axis_angle = (1./(2.*sin(angle+0.000001))) * axis_angle;
        //
        axis_angle(3) = angle;

          return axis_angle;

      }

      //
      Eigen::MatrixXd ComputeSkewSymmetricMatrix(Eigen::Vector3d positionVect)
      {
          
          // 
          Eigen::MatrixXd Skew_matrix(3,3);
          Skew_matrix.setZero();
          // 
          Skew_matrix(0,1) = -positionVect(2);
          Skew_matrix(0,2) =  positionVect(1);
          Skew_matrix(1,0) =  positionVect(2);
          Skew_matrix(1,2) = -positionVect(0);
          Skew_matrix(2,0) = -positionVect(1);
          Skew_matrix(2,1) =  positionVect(0);

          return Skew_matrix;
      }

      /** 
       * Compute the twsit transformation matrix associated with an homogenous transformation
       * expressed in the reference frame of the Homogenous matrix
       *
       */
      Eigen::MatrixXd ComputeTwistMatrix(Eigen::MatrixXd o_H_ee)
      {
          
          // 
          Eigen::MatrixXd TwistMx(6,6);
          TwistMx.setIdentity();
          // 
          TwistMx.block<3,3>(3,0) = ComputeSkewSymmetricMatrix(o_H_ee.block<3,1>(0,3));

          return TwistMx;
      }


      /** 
       * Compute the homogenous transformation related to a pose vector 
       * where the orientation is represented with axis angle representation
       */
      Eigen::MatrixXd PoseVector2HomogenousMx(Eigen::VectorXd Pose_vector)
      {
          
          // 
          Eigen::MatrixXd H(4,4);
          H.setIdentity();
          // 
          H.block<3,1>(0,3) = Pose_vector.head(3);
          H.block<3,3>(0,0) = AxisAngle2RotationMx(Pose_vector.tail(4));

          return H;
      }

      /** 
       * Compute the twist transformation from a rotation matrix and a position vector associated to
       * the origin and destination frame
       */
      Eigen::MatrixXd Get6DTwistTransform(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin)
      {
        // 6 x e transformation for 6D twist Vector [vx;vy;vz; wx;wy;wz]
        Eigen::MatrixXd d_X_o(6,6);
        d_X_o.setIdentity();

        d_X_o.block<3,3>(0,0) = destination_R_origin;
        d_X_o.block<3,3>(3,3) = destination_R_origin;
        d_X_o.block<3,3>(0,3) = -1.0 *ComputeSkewSymmetricMatrix(destination_P_origin) * destination_R_origin;

        return d_X_o;
      }

      /** 
       * Compute the wrench transformation from a rotation matrix and a position vector associated to
       * the origin and destination frame
       */
      Eigen::MatrixXd Get6DWrenchTransform(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin)
      {
        // 6 x e transformation for 6D Wrench Vector [fx;fy;fz; mx;my;mz]
        Eigen::MatrixXd d_Xstar_o(6,6);
        d_Xstar_o.setIdentity();

        d_Xstar_o.block<3,3>(0,0) = destination_R_origin;
        d_Xstar_o.block<3,3>(3,3) = destination_R_origin;
        d_Xstar_o.block<3,3>(3,0) = ComputeSkewSymmetricMatrix(destination_P_origin) * destination_R_origin;

        return d_Xstar_o;
      }

      /** 
       * Compute the wrench transformation from a rotation matrix and a position vector associated to
       * the origin and destination frame
       */
      Eigen::MatrixXd Compute6DWrenchMapping(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin)
      {
        // 6 x e transformation for 6D Wrench Vector [fx;fy;fz; mx;my;mz]
        Eigen::MatrixXd d_Xstar_o(6,6);
        d_Xstar_o.setIdentity();

        d_Xstar_o.block<3,3>(0,0) = destination_R_origin;
        d_Xstar_o.block<3,3>(3,3) = destination_R_origin;
        d_Xstar_o.block<3,3>(3,0) = ComputeSkewSymmetricMatrix(destination_P_origin) * destination_R_origin;

        return d_Xstar_o;
      }

      Eigen::VectorXd getMapped6DWrench(Eigen::VectorXd MappedWrench_o, Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin)
      {
        // 6 x e transformation for 6D Wrench Vector [fx;fy;fz; mx;my;mz]
        
        return Transformations::Compute6DWrenchMapping(destination_R_origin, destination_P_origin) * MappedWrench_o;

      }


};

class KinConverter
{
  public:

    KinConverter(){}
    ~KinConverter(){}

    MatrixXd yarpPose2eigenHmatrix(Vector yarpPoseVect)
    {
        MatrixXd H_out(4,4);
        H_out.setIdentity(4,4);

        H_out(0,3) = yarpPoseVect[0];
        H_out(1,3) = yarpPoseVect[1];
        H_out(2,3) = yarpPoseVect[2];

        yarp::sig::Matrix RotMx;
        RotMx = yarp::math::axis2dcm(yarpPoseVect.subVector(3,6));
        // extracting data from a Eigen matrix to a yarp matrix
        for (int row=0; row<3; row++)
        {
            for (int col=0; col<3; col++)
            {
                H_out(row, col) = RotMx(row,col);
            }
        }

        return H_out;

    }

};

// int mysign(double Val)
// {
//     return (0.0 < Val) - (Val < 0.0);
// }




class MatrixPseudoInverse
{


    public : 

        MatrixPseudoInverse(){}

        ~MatrixPseudoInverse(){}

        // Compute the pseudo inverse of a matrix
        template<typename _Matrix_Type_> _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
        {
            
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

            double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

            return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
        }

    

};

class MatrixPseudoInverse2
{

    public : 

        MatrixPseudoInverse2(){}

        ~MatrixPseudoInverse2(){}

        // Compute the pseudo inverse of a matrix
        template<typename _Matrix_Type_> _Matrix_Type_ get_pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
        {
            
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

            int svdSize = svd.singularValues().size();

            double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

            return svd.matrixV().leftCols(svdSize) *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().leftCols(svdSize).adjoint();
        }

};


// compute the integral of first order differential eq. using RK4
class firstOrderIntegrator
{

        double pole;
        double gain;
        double Ts;

        VectorXd init_fn;
        VectorXd init_fn2;
        VectorXd init_fn3;
        VectorXd init_fn4;
        VectorXd delta1;
        VectorXd delta2;
        VectorXd delta3;
        VectorXd delta4;
        VectorXd y_t;

    public:

       
        // 
        firstOrderIntegrator(double T, double gn, double pl, Eigen::VectorXd init_fn_val)
        {
            Ts = T;
            gain = gn;
            pole = pl;

            init_fn.resize(init_fn_val.rows(), init_fn_val.cols());
            init_fn2.resize(init_fn_val.rows(), init_fn_val.cols());
            init_fn3.resize(init_fn_val.rows(), init_fn_val.cols());
            init_fn4.resize(init_fn_val.rows(), init_fn_val.cols());

            delta1.resize(init_fn_val.rows(), init_fn_val.cols());
            delta2.resize(init_fn_val.rows(), init_fn_val.cols());
            delta3.resize(init_fn_val.rows(), init_fn_val.cols());
            delta4.resize(init_fn_val.rows(), init_fn_val.cols());

            y_t.resize(init_fn_val.rows(), init_fn_val.cols());


            init_fn = init_fn_val;

        }

         ~firstOrderIntegrator(){}

        VectorXd function_dot(double gn, double pl, const Eigen::VectorXd &init_fn_val, const Eigen::VectorXd &fn_t)
        {
            return - pl * init_fn_val + gn * fn_t;
        }

        // compute the integral of first order differential eq. using RK4
        VectorXd getRK4Integral(const Eigen::VectorXd &fn_t)
        {
    //
            delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);

            init_fn2 = init_fn + 0.5 * delta1;

            delta2   = Ts * function_dot(gain, pole, init_fn2, fn_t);

            init_fn3 = init_fn + 0.5 * delta2;

            delta3   = Ts * function_dot(gain, pole, init_fn3, fn_t);

            init_fn4 = init_fn + 0.5 * delta3;

            delta4   = Ts * function_dot(gain, pole, init_fn4, fn_t);

            // solution
            y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
            init_fn  = y_t;

            return y_t;

        }

        void setGain(double _gain)
        {
            gain = _gain;
        }

        void setPole(double _pole)
        {
            pole = _pole;
        }

        void setSampleTime(double T)
        {
            Ts = T;
        }

        
};
 
// Euler integrator
class EulerIntegrator
{
        double Ts;
        VectorXd funct_init;

    public:
        EulerIntegrator(double SamplingTime, VectorXd init_funct)
        {
            Ts = SamplingTime;
            //funct_init.resize(init_funct.rows());
            funct_init = init_funct;
        }
        ~EulerIntegrator() {}

        VectorXd getEulerIntegral(VectorXd funct)
        {
            VectorXd y_t = Ts *(funct_init + funct)/2.;
            funct_init = funct;

            return y_t;
        }


};

// Cubic polynomial Interpolator

class CubicInterpolator
{
    public:

    int nb_of_points;
    VectorXd Down_Index;
    VectorXd Up_Index;

    CubicInterpolator() {}
    ~CubicInterpolator(){}

    void DecreasingInterpolation(int nbpts)
    {
        nb_of_points = nbpts;
        Down_Index.resize(nb_of_points);
        Down_Index.setZero(nb_of_points);

        if (nb_of_points == 1)
        {
            Down_Index(0)= 1. ;
        }
        else
        {
            for(int i=0; i<nb_of_points; i++)
            {
                Down_Index(i) =  1.
                               -(3.*pow(i,2.)/pow(nb_of_points-1, 2.)
                               - 2.*pow(i,3.)/pow(nb_of_points-1, 3.));
            }
        }



    }

    void IncreasingInterpolation(int nbpts)
    {
        nb_of_points = nbpts;
        Up_Index.resize(nb_of_points);
        Up_Index.setZero(nb_of_points);

        if (nb_of_points == 1)
        {
            Down_Index(0)= 1. ;
        }
        else
        {
            for(int i=0; i<nb_of_points; i++)
            {
                Up_Index(i) =  3.*pow(i,2.)/pow(nb_of_points-1, 2.)
                              -2.*pow(i,3.)/pow(nb_of_points-1, 3.);
            }
        }


    }

};


// Kalman filter
class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(   double dt,
                  const Eigen::MatrixXd& A,
                  const Eigen::MatrixXd& C,
                  const Eigen::MatrixXd& Q,
                  const Eigen::MatrixXd& R,
                  const Eigen::MatrixXd& P
                                            ): A(A), C(C), Q(Q), R(R), P0(P),
                                               m(C.rows()), n(A.rows()), dt(dt), initialized(false),
                                               I(n, n), x_hat(n), x_hat_new(n)
    {
      I.setIdentity();
    }

  /**
  * Create a blank estimator.
  */
  KalmanFilter(){}

  /**
  * Initialize the filter with initial states as zero.
  */
  void init(){
      x_hat.setZero();
      P = P0;
      t0 = 0;
      t = t0;
      initialized = true;
    }

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0){
      x_hat = x0;
      P = P0;
      this->t0 = t0;
      t = t0;
      initialized = true;
    }
  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& y){

      if(!initialized)
        throw std::runtime_error("Filter is not initialized!");

      x_hat_new = A * x_hat;
      P = A*P*A.transpose() + Q;
      K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
      x_hat_new += K * (y - C*x_hat_new);
      P = (I - K*C)*P;
      x_hat = x_hat_new;

      t += dt;
    }
  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A){

      this->A = A;
      this->dt = dt;
      update(y);
    }
  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };

  void setState(const Eigen::VectorXd& x_hat_1)
  {
        x_hat = x_hat_1;
  }

private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};


// Kalman filter
class multiSignalKalmanFilter {

  private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // number of signals
    int nSig;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states of one signal
    Eigen::VectorXd x_hat, x_hat_new, x_hat0;

    // Estimated states of all signals
    Eigen::VectorXd nX_hat, nX_hat_new, nX_hat0;

    // Gain Matrix for the computation od all signals
    Eigen::MatrixXd A_nSig, C_nSig, K_nSig;

  public:

    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    multiSignalKalmanFilter(         double dt,
                        const Eigen::MatrixXd& A,
                        const Eigen::MatrixXd& C,
                        const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& R,
                        const Eigen::MatrixXd& P,
                                   int Ns
                                                  ): A(A), C(C), Q(Q), R(R), P0(P),
                                                     m(C.rows()), n(A.rows()), dt(dt), initialized(false),
                                                     I(n, n), x_hat(n), x_hat_new(n), x_hat0(n), nSig(Ns), nX_hat(nSig*n), nX_hat_new(nSig*n), nX_hat0(nSig*n)
      {
        I.setIdentity();
      }

    /**
    * Create a blank estimator.
    */
    multiSignalKalmanFilter(){}

    /**
    * Initialize the filter with initial states as zero.
    */
    void init(){
        x_hat.setZero();
        P = P0;
        t0 = 0;
        t = t0;
        initialized = true;
      }

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0){
        
      // multi Signal
        nX_hat = x0;
        nX_hat0 = nX_hat;

        x_hat   = nX_hat.segment(0, n);
        x_hat0 = nX_hat0.segment(0, n);

        P = P0;
        this->t0 = t0;
        t = t0;
        initialized = true;



        // Dynamic matrix
        A_nSig.resize(nSig*n, nSig*n);
        
        // Observation vector
        C_nSig.resize(nSig*m, nSig*n);
        
        // kalman gain matrix
        K_nSig.resize(nSig*n, nSig*m);

      }
    /**
    * Update the estimated state based on measured values. The
    * time step is assumed to remain constant.
    */
    void update(const Eigen::VectorXd& y){

        if(!initialized)
          throw std::runtime_error("Filter is not initialized!");

        t += dt;
        // if(t==dt)
        // {
        //     x_hat(0) =  y(0);
        //     x_hat(1) = (y(0) - x_hat0(0))/dt;
        // }

        x_hat_new = A * x_hat;
        P = A*P*A.transpose() + Q;
        K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
        x_hat_new += K * (y.segment(0, m) - C*x_hat_new);
        P = (I - K*C)*P;
        x_hat = x_hat_new;
        

      // computing for all other signals
      // update the dynamic matrix
        for (int i=0; i<nSig; i++)
        {
            A_nSig.block(n*i, n*i, n, n) = A;
        }
      // update of the matrix of observation vectors
        for (int i=0; i<nSig; i++)
        {
            C_nSig.block(m*i, n*i, m, n) = C;
        }

      // update the Kalman gain matrix
        for (int i=0; i<nSig; i++)
        {
            K_nSig.block(n*i, m*i, n, m) = K;
        }

        // if(t == dt)
        // {
        //     for (int i=0; i<nSig; i++)
        //     {
        //         nX_hat(n*i) = y(i);
        //         nX_hat(n*i+1) = (y(i) - nX_hat0(n*i))/dt;
        //     }
        // }
        
        nX_hat_new = A_nSig * nX_hat;

        nX_hat_new += K_nSig * (y - C_nSig * nX_hat_new);

        nX_hat = nX_hat_new;


      }
    /**
    * Update the estimated state based on measured values,
    * using the given time step and dynamics matrix.
    */
    void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A){

        this->A = A;
        this->dt = dt;
        update(y);
      }
    /**
    * Return the current state and time.
    */
    Eigen::VectorXd state() { return nX_hat; };
    double time() { return t; };

    void setState(const Eigen::VectorXd& nXhat_1)
    {
          nX_hat = nXhat_1;
    }
    int get_nSig()
    {
      return nSig;
    }
};


// Dynamical System solver

class DynamicSystemSolver
{

        MatrixXd DynMx;
        VectorXd InVec;
        double      Ts;

        VectorXd init_fn;
        VectorXd init_fn2;
        VectorXd init_fn3;
        VectorXd init_fn4;
        VectorXd delta1;
        VectorXd delta2;
        VectorXd delta3;
        VectorXd delta4;
        VectorXd y_t;

    public:

       
        // 
        DynamicSystemSolver() {}

         ~DynamicSystemSolver(){}

        void InitializeDynamics(double T, MatrixXd A, VectorXd B, const Eigen::VectorXd &init_fn_val)
        {
            Ts = T;
            DynMx = A;
            InVec = B;

            init_fn.resize(DynMx.rows());
            init_fn2.resize(DynMx.rows());
            init_fn3.resize(DynMx.rows());
            init_fn4.resize(DynMx.rows());

            delta1.resize(DynMx.rows());
            delta2.resize(DynMx.rows());
            delta3.resize(DynMx.rows());
            delta4.resize(DynMx.rows());

            y_t.resize(DynMx.rows());


            init_fn = init_fn_val;

        }

        VectorXd function_dot(MatrixXd A, VectorXd B, const Eigen::VectorXd &init_fn_val, const double &fn_t)
        {
            return A * init_fn_val + B * fn_t;
        }

        // compute the integral of first order differential eq. using RK4
        VectorXd getRK4Solution(const double &fn_t)
        {
    //
            delta1   = Ts * function_dot(DynMx, InVec, init_fn, fn_t);

            init_fn2 = init_fn + 0.5 * delta1;

            delta2   = Ts * function_dot(DynMx, InVec, init_fn2, fn_t);

            init_fn3 = init_fn + 0.5 * delta2;

            delta3   = Ts * function_dot(DynMx, InVec, init_fn3, fn_t);

            init_fn4 = init_fn + 0.5 * delta3;

            delta4   = Ts * function_dot(DynMx, InVec, init_fn4, fn_t);

            // solution
            y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
            init_fn  = y_t;

            return y_t;

        }

        void updateFirstState(VectorXd FirstState)
        {
            // 
            if (FirstState.rows() != DynMx.rows()/2)
            {
              printf("Error dimensions don't agree \n");
              return;
            }
            init_fn.segment(DynMx.rows()/2, 0) = FirstState;
        }

        void set_InputVector(VectorXd _InVec)
        {
            InVec = _InVec;
        }

        void set_DynamicMatrix(MatrixXd _DynMx)
        {
            DynMx = _DynMx;
        }

        Eigen::MatrixXd get_DynamicMatrix()
        {
            return DynMx;
        }


        void setSampleTime(double T)
        {
            Ts = T;
        }

        
};

// with and input vector
class DynamicSystemSolver2
{

        MatrixXd DynMx;
        MatrixXd InVec;
        double      Ts;

        VectorXd init_fn;
        VectorXd init_fn2;
        VectorXd init_fn3;
        VectorXd init_fn4;
        VectorXd delta1;
        VectorXd delta2;
        VectorXd delta3;
        VectorXd delta4;
        VectorXd y_t;

    public:

       
        // 
        DynamicSystemSolver2() {}

         ~DynamicSystemSolver2(){}

        void InitializeDynamics(double T, MatrixXd A, MatrixXd B, Eigen::VectorXd init_fn_val)
        {
            Ts = T;
            DynMx = A;
            InVec = B;

            init_fn.resize(DynMx.rows());
            init_fn2.resize(DynMx.rows());
            init_fn3.resize(DynMx.rows());
            init_fn4.resize(DynMx.rows());

            delta1.resize(DynMx.rows());
            delta2.resize(DynMx.rows());
            delta3.resize(DynMx.rows());
            delta4.resize(DynMx.rows());

            y_t.resize(DynMx.rows());


            init_fn = init_fn_val;

        }

        VectorXd function_dot(MatrixXd A, MatrixXd B, Eigen::VectorXd init_fn_val, Eigen::VectorXd fn_t)
        {
            return A * init_fn_val + B * fn_t;
        }

        // compute the integral of first order differential eq. using RK4
        VectorXd getRK4Solution(Eigen::VectorXd fn_t)
        {
    //
            delta1   = Ts * function_dot(DynMx, InVec, init_fn, fn_t);

            cout << " delta 1 " << endl;

            init_fn2 = init_fn + 0.5 * delta1;

            delta2   = Ts * function_dot(DynMx, InVec, init_fn2, fn_t);

            

            init_fn3 = init_fn + 0.5 * delta2;

            delta3   = Ts * function_dot(DynMx, InVec, init_fn3, fn_t);
            cout << " delta 3 " << endl;

            init_fn4 = init_fn + 0.5 * delta3;
            cout << " delta 4 " << endl;

            delta4   = Ts * function_dot(DynMx, InVec, init_fn4, fn_t);
            cout << " delta 5 " << endl;

            // solution
            y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
            init_fn  = y_t;

            return y_t;

        }

        void updateFirstState(VectorXd FirstState)
        {
            // 
            if (FirstState.rows() != DynMx.rows()/2)
            {
              printf("Error dimensions don't agree \n");
              return;
            }
            init_fn.segment(DynMx.rows()/2, 0) = FirstState;
        }

        void set_ControlMatrix(MatrixXd _InVec)
        {
            InVec = _InVec;
        }

        void set_DynamicMatrix(MatrixXd _DynMx)
        {
            DynMx = _DynMx;
        }

        Eigen::MatrixXd get_DynamicMatrix()
        {
            return DynMx;
        }


        void setSampleTime(double T)
        {
            Ts = T;
        }

        
};


// Dynamic Filter for the CoM based on the ZMP
// class CpDynamicFilter
class ZmpDynamicFilter
{

  // for the Dynamic Filter
           MatrixXd PzsDf;  
           MatrixXd PzuDf;
           
           MatrixXd PzuDf_T_PzsDf;
           VectorXd PzuDf_T_ones_ns1;
           
           double alpha;
           double gamma;
           



       public:

        MatrixXd Qtrl;
        MatrixXd invQtrl;
        VectorXd VPx;
        VectorXd VPy;
        VectorXd xTild;
        VectorXd yTild;

        VectorXd xOptimalSol;
        VectorXd yOptimalSol;
        double xDfOutput;
        double yDfOutput;

        ZmpDynamicFilter() {}

        ~ZmpDynamicFilter() {}

        void InitializeDynamicFilter(Discrete_CP_LIP_Model *DMod,         // Discrete LIPM Object)
                                         MpcBased_CP_Model *MpcModel,     // LMPC model of the LIPM
                                                    double GainsArray[],  // gains of the QP
                                                       int nbSampStp[])   // array of nb of samples per steps)  // Gain of the QP
        {

          
          // gains
          alpha = GainsArray[0];  // gain related to the minizing the control effort (CoP)
          gamma = GainsArray[2];  // gain related to the Cp tracking
          
          // Computation of the An and CAn-1 Matrices
          
          // initialization of the states  [x xdot xddot]
          xTild.resize(3);    xTild.setZero(3);
          yTild.resize(3);    yTild.setZero(3);
         
          int ns1 = nbSampStp[0];

          // Initialization for memo allocation
          PzsDf.resize(ns1,3);      PzsDf.setZero(ns1,3);
          PzuDf.resize(ns1,ns1);    PzuDf.setZero(ns1,ns1);



          PzsDf  = MpcModel->Pzs.block(0, 0, ns1,3);
          PzuDf  = MpcModel->Pzu.block(0, 0, ns1,ns1);
         
          VectorXd ones_ns1(ns1);
          ones_ns1.setOnes(ns1);

           // prefactorization
           PzuDf_T_PzsDf    = PzuDf.transpose()  * PzsDf;
           PzuDf_T_ones_ns1 = PzuDf.transpose()  * ones_ns1;
          
           // precomputation of the Hessian Matrix Qtrl
           Qtrl = gamma * PzuDf.transpose() * PzuDf + alpha * MatrixXd::Identity(ns1,ns1);
           // inverse of Qtrl
           invQtrl = Qtrl.inverse();

           
           //cout << " size of invQtrl  is : \n" << invQtrl.rows() << " and " << invQtrl.cols()<< endl;

        }

        void UpdateDynFilterStates(Discrete_CP_LIP_Model *DMod,      // Discrete LIPM Object)
                                                  double xZmp_error,  // error on the CP in X direction
                                                  double yZmp_error)  // error on the CP in Y direction 
        {
          
          double xRef_ZmpError, yRef_ZmpError;

          xRef_ZmpError = xZmp_error;
          yRef_ZmpError = yZmp_error;

          // X direction
          VPx = gamma * (PzuDf_T_PzsDf * xTild - PzuDf_T_ones_ns1 * xRef_ZmpError);

          // 
          xOptimalSol = - invQtrl * VPx;

          // Update the LIPM model associate with the filter
          xTild = DMod->MxA_R * xTild + DMod->VeB_R * xOptimalSol(0);
          xDfOutput = DMod->VeCz * xTild;

          
          // Y direction
          VPy = gamma * (PzuDf_T_PzsDf * yTild - PzuDf_T_ones_ns1 * yRef_ZmpError);
          // 
          yOptimalSol = - invQtrl * VPy;

          // Update the LIPM model associate with the filter
          yTild = DMod->MxA_R * yTild + DMod->VeB_R * yOptimalSol(0);
          yDfOutput = DMod->VeCz * yTild;

        }

        void ReinitializeDfStates()
        {
            xTild.setZero();
            yTild.setZero();
        }

        double getDynFilterOutputX()
        {
          return xDfOutput;
        }

        double getDynFilterOutputY()
        {
          return yDfOutput;
        }

        VectorXd getDynFilerStatesX()
        {
          return xTild;
        }

        VectorXd getDynFilerStatesY()
        {
          return yTild;
        }
};


// class to transform wrench from the sensor frame to the reference frame
class WrenchTransformation
{
  
  public:

    WrenchTransformation(){}
    ~WrenchTransformation(){}

    MatrixXd ComputeWrenchTransforms(MatrixXd HTrsf_sensor2RefFrame)
    {
      // Force/Torque transformation matrices
          // skew symmetric transformation matrices
      Eigen::MatrixXd SkewMx_FTsensor_RefFrame(3,3);

      SkewMx_FTsensor_RefFrame.setZero(3,3);
      SkewMx_FTsensor_RefFrame(0,1) = -HTrsf_sensor2RefFrame(2,3);
      SkewMx_FTsensor_RefFrame(0,2) =  HTrsf_sensor2RefFrame(1,3);
      SkewMx_FTsensor_RefFrame(1,0) =  HTrsf_sensor2RefFrame(2,3);
      SkewMx_FTsensor_RefFrame(1,2) = -HTrsf_sensor2RefFrame(0,3);
      SkewMx_FTsensor_RefFrame(2,0) = -HTrsf_sensor2RefFrame(1,3);
      SkewMx_FTsensor_RefFrame(2,1) =  HTrsf_sensor2RefFrame(0,3);

      // wrench transformation matrices
      MatrixXd WrenchTrsf_FTsensor2RefFrame;
      WrenchTrsf_FTsensor2RefFrame.resize(6,6); WrenchTrsf_FTsensor2RefFrame.setZero(6,6);

        // Wrench transformation from the sensor frame to the reference frame
      WrenchTrsf_FTsensor2RefFrame.block(0,0, 3,3) = HTrsf_sensor2RefFrame.block(0,0, 3,3);
      WrenchTrsf_FTsensor2RefFrame.block(3,0, 3,3) = SkewMx_FTsensor_RefFrame * HTrsf_sensor2RefFrame.block(0,0, 3,3);
      WrenchTrsf_FTsensor2RefFrame.block(3,3, 3,3) = HTrsf_sensor2RefFrame.block(0,0, 3,3);

      return WrenchTrsf_FTsensor2RefFrame;

    }
};

#endif // CpMath_Utilities
