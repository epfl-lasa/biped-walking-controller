
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

/*
 * CP_QPSolver_OASES : This class encodes a QP solver based on 
 * qpOASES library 
 *
*/

#ifndef OptimalFilters_H
#define OptimalFilters_H

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "CpMath_Utilities.h"


class KF_Velo_mAccelCldNoise
{
	public :

	KalmanFilter *KF_Velo;

	KF_Velo_mAccelCldNoise(){}

	~KF_Velo_mAccelCldNoise()
	{
		close();
	}

	void Create_Velo_mAccelCldNoiseKF(double _dt, double _rho, double _sigmaQ1, double _sigmaQ2, double _sigmaR)
	{
		// creating a three state Kalman filter with colored noise
		int n = 3; // Number of states
        int m = 1; // Number of measurements

        double dt      = _dt; 		// Time step
        double rho     = _rho;  	//degree of correlation between successive acceleration
        double sigmaQ1 = _sigmaQ1;
        double sigmaQ2 = _sigmaQ2;
        double sigmaR  = _sigmaR;

        Eigen::MatrixXd A(n, n); // System dynamics matrix
        Eigen::MatrixXd C(m, n); // Output matrix
        Eigen::MatrixXd Q(n, n); // Process noise covariance
        Eigen::MatrixXd R(m, m); // Measurement noise covariance
        Eigen::MatrixXd P(n, n); // Estimate error covariance

        // Discrete LTI projectile motion, measuring position only
        A << 1, 1, dt, 0, rho, 0, 0, 0, 1;
        C << 0, 0, 1;

        // Reasonable covariance matrices
        Q <<  .0, .0, .0, .0, sigmaQ1, .0, .00, .0, sigmaQ2;
        R << sigmaR;

              
        P(0,0) = sigmaR*sigmaR;       P(0,1) = 0.0;                                       P(0,2) = sigmaR*sigmaR/dt;
        P(1,0) = 0.0;                 P(1,1) = sigmaQ1*sigmaQ1/(1-rho*rho);               P(1,2) = -rho*sigmaQ1*sigmaQ1/((1-rho*rho)*dt);
        P(2,0) = sigmaR*sigmaR/dt;    P(2,1) = -rho*sigmaQ1*sigmaQ1/((1-rho*rho)*dt);     P(2,2) = (2.*sigmaR*sigmaR +sigmaQ1*sigmaQ1/(1-rho*rho))/dt;


        KF_Velo = new KalmanFilter(dt, A, C, Q, R, P);
	}



	void init(double t0, Eigen::VectorXd X0)
	{
		KF_Velo->init(t0, X0);
	}

	void update(Eigen::VectorXd m_Xk)
	{
		KF_Velo->update(m_Xk);
	}
	Eigen::VectorXd state()
	{
		return KF_Velo->state();
	}

	void close()
	{
		if (KF_Velo) {
    		delete KF_Velo;
    		KF_Velo = 0;
		}
	}


};

class KF_Pos_mVeloWhtNoise
{

	public :

	KalmanFilter *KF_Pos;

	KF_Pos_mVeloWhtNoise() {}

	~KF_Pos_mVeloWhtNoise()
	{
		close();
	}

	void Create_Pos_mVeloWhtNoiseKF(double _dt, double _sigmaQ1, double _sigmaQ2, double _sigmaR)
	{
		
		int n = 2; // Number of states
        int m = 1; // Number of measurements
        double dt = _dt;
        double sigmaQ1 = _sigmaQ1;  // 0.05
        double sigmaQ2 = _sigmaQ2;
        double sigmaR  = _sigmaR;   // 2.00

        Eigen::MatrixXd A2(n, n); // System dynamics matrix
        Eigen::MatrixXd C2(m, n); // Output matrix
        Eigen::MatrixXd Q2(n, n); // Process noise covariance
        Eigen::MatrixXd R2(m, m); // Measurement noise covariance
        Eigen::MatrixXd P2(n, n); // Estimate error covariance

        A2 << 1, dt, 0, 1;
        C2 << 0, 1;

        // Reasonable covariance matrices
        Q2(0,0) =  sigmaQ1*sigmaQ1*dt*dt*dt/3.;   Q2(0,1) =  sigmaQ1*sigmaQ2*dt*dt/2.;
        Q2(1,0) =  sigmaQ1*sigmaQ2*dt*dt/2.;      Q2(1,1) =  sigmaQ2*sigmaQ2*dt;

        P2(0,0) =  sigmaR*sigmaR;             	  P2(0,1) =  sigmaR*sigmaR/(2.*dt);
        P2(1,0) =  sigmaR*sigmaR/(2.*dt);     	  P2(1,1) =  2./3.*sigmaQ2*sigmaQ2*dt+sigmaR*sigmaR/(2.*dt*dt);

        R2 << sigmaR;

             
        KF_Pos = new KalmanFilter(dt, A2, C2, Q2, R2, P2);
	}

	void init(double t0, Eigen::VectorXd X0)
	{
		KF_Pos->init(t0, X0);
	}

	void update(Eigen::VectorXd m_Xk)
	{
		KF_Pos->update(m_Xk);
	}
	Eigen::VectorXd state()
	{
		return KF_Pos->state();
	}

	void close()
	{
		if (KF_Pos) {
    		delete KF_Pos;
    		KF_Pos = 0;
		}
	}

};


class KF_PosiVelo_mPosi_WhtNoise
{

	public :

	multiSignalKalmanFilter *KF_Pos;

	KF_PosiVelo_mPosi_WhtNoise(){}

	~KF_PosiVelo_mPosi_WhtNoise()
	{
		close();
	}

	void Create_PosiVelo_mPosi_WhtNoiseKF(double _dt, double _sigmaQ1, double _sigmaQ2, double _sigmaR, int _Ns)
	{
		
		int n = 2; // Number of states
        int m = 1; // Number of measurements
        double dt = _dt;
        double sigmaQ1 = _sigmaQ1;  // 0.05
        double sigmaQ2 = _sigmaQ2;
        double sigmaR  = _sigmaR;   // 2.00

        Eigen::MatrixXd A2(n, n); // System dynamics matrix
        Eigen::MatrixXd C2(m, n); // Output matrix
        Eigen::MatrixXd Q2(n, n); // Process noise covariance
        Eigen::MatrixXd R2(m, m); // Measurement noise covariance
        Eigen::MatrixXd P2(n, n); // Estimate error covariance


        A2 << 1, dt, 0, 1;
        C2 << 1, 0;

        // Reasonable covariance matrices
        Q2(0,0) =  sigmaQ1*sigmaQ1*dt*dt*dt/3.;   Q2(0,1) =  sigmaQ1*sigmaQ2*dt*dt/2.;
        Q2(1,0) =  sigmaQ1*sigmaQ2*dt*dt/2.;      Q2(1,1) =  sigmaQ2*sigmaQ2*dt;

        P2(0,0) =  sigmaR*sigmaR;             	  P2(0,1) =  sigmaR*sigmaR/(2.*dt);
        P2(1,0) =  sigmaR*sigmaR/(2.*dt);     	  P2(1,1) =  2./3.*sigmaQ2*sigmaQ2*dt+sigmaR*sigmaR/(2.*dt*dt);

        R2 << sigmaR*sigmaR;

        KF_Pos = new multiSignalKalmanFilter(dt, A2, C2, Q2, R2, P2, _Ns);
	}



	void init(double t0, Eigen::VectorXd X0)
	{
		KF_Pos->init(t0, X0);
	}

	void update(Eigen::VectorXd m_Xk)
	{
		KF_Pos->update(m_Xk);
	}

	Eigen::VectorXd fPosition()
	{
		
		VectorXd xPos(KF_Pos->get_nSig());

		for (int i=0; i<KF_Pos->get_nSig(); i++)
		{
			xPos(i) = (KF_Pos->state())(2*i);
		}
		return xPos;	
	}

	Eigen::VectorXd fVelocity()
	{
		VectorXd xVelo(KF_Pos->get_nSig());

		for (int i=0; i<KF_Pos->get_nSig(); i++)
		{
			xVelo(i) = (KF_Pos->state())(2*i+1);
		}

		return xVelo;
	}

	VectorXd fState()
	{
		return KF_Pos->state();
	}


	void close()
	{
		if (KF_Pos) {
    		delete KF_Pos;
    		KF_Pos = 0;
		}
	}


};

// constant velocity with colored noise
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class KF_Velo_mVelo_ColoredNoise
{

	public :

	multiSignalKalmanFilter *KF_Pos;

	KF_Velo_mVelo_ColoredNoise(){}

	~KF_Velo_mVelo_ColoredNoise()
	{
		close();
	}

	void Create_KF_Velo_mVelo_ColoredNoiseKF(double _dt, double _sigmaQ1, double _sigmaR, double _rho, int _Ns)
	{
		
		int n = 2; // Number of states
        int m = 1; // Number of measurements
        double dt = _dt;
        double sigmaQ1 = _sigmaQ1;  // 0.05
        double sigmaR  = _sigmaR;   // 2.00
        double rho 	   = _rho;

        Eigen::MatrixXd A2(n, n); // System dynamics matrix
        Eigen::MatrixXd C2(m, n); // Output matrix
        Eigen::MatrixXd Q2(n, n); // Process noise covariance
        Eigen::MatrixXd R2(m, m); // Measurement noise covariance
        Eigen::MatrixXd P2(n, n); // Estimate error covariance


        A2 << 1, 1, 0, rho;
        C2 << 1, 0;

        // Reasonable covariance matrices
        Q2(0,0) = 0.0;   Q2(0,1) =  0.0;
        Q2(1,0) = 0.0;   Q2(1,1) =  sigmaQ1*sigmaQ1;

        P2(0,0) =  sigmaR*sigmaR;       P2(0,1) =  0.0;
        P2(1,0) =  0.0;     	  		P2(1,1) =  sigmaQ1*sigmaQ1/(1- rho*rho);

        R2 << sigmaR*sigmaR;

        KF_Pos = new multiSignalKalmanFilter(dt, A2, C2, Q2, R2, P2, _Ns);
	}



	void init(double t0, Eigen::VectorXd X0)
	{
		KF_Pos->init(t0, X0);
	}

	void update(Eigen::VectorXd m_Xk)
	{
		KF_Pos->update(m_Xk);
	}

	Eigen::VectorXd fVelocity()
	{
		
		VectorXd xPos(KF_Pos->get_nSig());

		for (int i=0; i<KF_Pos->get_nSig(); i++)
		{
			xPos(i) = (KF_Pos->state())(2*i);
		}
		return xPos;	
	}

	// Eigen::VectorXd fVelocity()
	// {
	// 	VectorXd xVelo(KF_Pos->get_nSig());

	// 	for (int i=0; i<KF_Pos->get_nSig(); i++)
	// 	{
	// 		xVelo(i) = (KF_Pos->state())(2*i+1);
	// 	}

	// 	return xVelo;
	// }

	VectorXd fState()
	{
		return KF_Pos->state();
	}


	void close()
	{
		if (KF_Pos) {
    		delete KF_Pos;
    		KF_Pos = 0;
		}
	}


};


// Legs filters
class multi_KF_Pos_mVeloWhtNoise
{

	public :

	multiSignalKalmanFilter *KF_Pos;

	multi_KF_Pos_mVeloWhtNoise() {}

	~multi_KF_Pos_mVeloWhtNoise()
	{
		close();
	}

	void Create_multi_KF_Pos_mVeloWhtNoiseKF(double _dt, double _sigmaQ1, double _sigmaQ2, double _sigmaR, int _Ns)
	{
		
		int n = 2; // Number of states
        int m = 1; // Number of measurements
        double dt = _dt;
        double sigmaQ1 = _sigmaQ1;  // 0.05
        double sigmaQ2 = _sigmaQ2;
        double sigmaR  = _sigmaR;   // 2.00

        Eigen::MatrixXd A2(n, n); // System dynamics matrix
        Eigen::MatrixXd C2(m, n); // Output matrix
        Eigen::MatrixXd Q2(n, n); // Process noise covariance
        Eigen::MatrixXd R2(m, m); // Measurement noise covariance
        Eigen::MatrixXd P2(n, n); // Estimate error covariance

        A2 << 1, dt, 0, 1;
        C2 << 0, 1;

        // Reasonable covariance matrices
        Q2(0,0) =  sigmaQ1*sigmaQ1*dt*dt*dt/3.;   Q2(0,1) =  sigmaQ1*sigmaQ2*dt*dt/2.;
        Q2(1,0) =  sigmaQ1*sigmaQ2*dt*dt/2.;      Q2(1,1) =  sigmaQ2*sigmaQ2*dt;

        P2(0,0) =  sigmaR*sigmaR;             	  P2(0,1) =  sigmaR*sigmaR/(2.*dt);
        P2(1,0) =  sigmaR*sigmaR/(2.*dt);     	  P2(1,1) =  2./3.*sigmaQ2*sigmaQ2*dt+sigmaR*sigmaR/(2.*dt*dt);

        R2 << sigmaR*sigmaR;

             
         KF_Pos = new multiSignalKalmanFilter(dt, A2, C2, Q2, R2, P2, _Ns);
	}



	void init(double t0, Eigen::VectorXd X0)
	{
		KF_Pos->init(t0, X0);
	}

	void update(Eigen::VectorXd m_Xk)
	{
		KF_Pos->update(m_Xk);
	}

	Eigen::VectorXd fPosition()
	{
		
		VectorXd xPos(KF_Pos->get_nSig());

		for (int i=0; i<KF_Pos->get_nSig(); i++)
		{
			xPos(i) = (KF_Pos->state())(2*i);
		}
		return xPos;	
	}

	Eigen::VectorXd fVelocity()
	{
		VectorXd xVelo(KF_Pos->get_nSig());

		for (int i=0; i<KF_Pos->get_nSig(); i++)
		{
			xVelo(i) = (KF_Pos->state())(2*i+1);
		}

		return xVelo;
	}

	VectorXd fState()
	{
		return KF_Pos->state();
	}


	void close()
	{
		if (KF_Pos) {
    		delete KF_Pos;
    		KF_Pos = 0;
		}
	}

};


#endif // OptimalFilters


