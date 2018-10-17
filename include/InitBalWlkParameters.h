#ifndef InitBalWlkParameters_H
#define InitBalWlkParameters_H

#include <string>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


typedef Eigen::Matrix<double, 6, 1> Vector6d;


class InitBalWlkParameters
{

   public:

    // Name of the robot 
    string      RobotName;

    double    SamplingTime;
    const double    Gravity  = 9.81;
    double    CoMHeight;

    /* number of sample per steps
       nsp  : number of samples in the current step
       nsp1 : number of samples in the 1st predicted step
       nsp2 : number of samples in the 2nd and 3rd predicted steps
       We have assigned the same number of samples per step in the 2nd
       and 3rd predicted step */
    int       nsp  ;
    int       nsp1 ;
    int       nsp2 ;

    // Total mass
    double    TotalMass;

    // Duration of each step over the horizon
    /* [1] : duration of step 1
     * [2] : duration of step 2
     * [3] : duration of step 3
     * [4] : duration of step 4
     * [5] : duration of double support phase */
    double    DurationSteps[5];

    // nb of sample for each step
    int       SamplesPerStep[4];

    // Control horizon
    int       CtrlHorizon;

    // gains of the QP [alpha, beta, gamma, kappa,  alpha_R, beta_R, gamma_R, kappa_R]
    double    gains[8];

    // Type of inverted Pendulum Dynamic model
    int       CpModelOption;

    // Inverted pendulum natural frequency
    double    OmegaZero;

    // array of stance foot state {Left, Right}
    int       StanceIndicator[2];

    // constraint on maximum height of swing foot
    double    maxFootHeight ;

    // vector of Foosteps constraints
    VectorXd  SupportConstraints;

    // translation vector from Base to the CoM
    VectorXd  Translation_B_CoM;

    // Initial velocity
    Vector3d  InitialVelocity;
    // Set initial stance foot positon and orientation
    VectorXd  InitCoPrefPose;

    // Vector of samples
    VectorXd SamplesVector;

    // Vectror of positions of support foot edge
    VectorXd SupportFootEdgePositions;

    // Matrix containing the normal vectors to the support foot edges
    MatrixXd SupportFootEdgeNormals;

    // footsteps constraints only or footsteps and ZMP constraints
    bool  FootStepsCnstrOnly;

    // compensation of the Base orientation using the  IMU
    bool IMU_reference;

    double Accel_factor;

    double Speed_factor;

    // number of sampling instant within a step
    int num_samples;

    // force threshold for support phase
    double Threshold_SSP;

    // send commands to the robot
    bool Command_exe;
    // sending commands for the grasping
    bool ExecuteGrasping;

    // sending commands for the grasping
    bool ExecuteWalking;

    //Switching Synchronization with FT sensor
    bool SwitchingSync;

    // variable for force torque feedback
    int FT_feedback;
    //
    bool calibration;

    // Parameters for the commands writer
    bool useOffset;
    bool SmoothCmd;
    int  NbInterpolationPts;

    // weighting factors from simulator to real robot
    double HipRollFactor;
    double HipYawFactor;
    double AnkleRollFactor;

     // additional parameters for the arms motion
    bool VelocityIK;
    double Delta_theta_max;

    // gain vector for the servoing task of the hands
    VectorXd Gain_hand;

    // Threshold for the velocities in the state to input compensator
    Vector6d VeloFeedbackThreshold;

    // arms admittance parameters
    // Properties of the admittance law
    Vector6d lh_virtualInertia;
    Vector6d rh_virtualInertia;

    Vector6d lh_virtualDamping;
    Vector6d rh_virtualDamping;

    Vector6d lh_virtualStiffness;
    Vector6d rh_virtualStiffness;

    //
     // Defaut consturctor and Destructor
    InitBalWlkParameters(int period, const std::string robotName, int ForceFeedback);

    ~InitBalWlkParameters();



};

#endif // InitBalWlkParameters_H
