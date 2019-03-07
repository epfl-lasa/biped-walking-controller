#include "DesVelocityCommand_lpvDS.h"

DesVelocityCommand_lpvDS::DesVelocityCommand_lpvDS(string moduleName, string robotName, int DSType, string path_to_models, string lpvDS_model_name)
                                                    : moduleName_(moduleName)
                                                    , robotName_(robotName)
                                                    , DSType_(DSType)
                                                    , path_to_models_(path_to_models)
                                                    , lpvDS_model_name_(lpvDS_model_name) {}


bool DesVelocityCommand_lpvDS::initReader(){

    des_com_vel_.resize(3);
    des_com_vel_.setZero();
    
    // ====== Opening the port for Root-Link in World (CoM) Pose reader w/robotName ====== //
    std::string RootlinkPose_portName="/";
    RootlinkPose_portName += robotName_;
    RootlinkPose_portName += "/get_root_link_WorldPose:o";


    RootlinkPose_port_In.open("/RootlinkPose_In:i");
    if(!Network::connect(RootlinkPose_portName.c_str(), RootlinkPose_port_In.getName().c_str())){
        printf(" Unable to connect to Root-link pose reader port");
        return false;
    }

    // ====== Opening the port to publish the Aligned CoM in position+quaternion ====== //
    std::string CoMPortName = "/";
    CoMPortName += robotName_;
    CoMPortName += "/CoMPose:o";

    CoMPose_port_Out.open(CoMPortName.c_str());
    CoMPoses.resize(7, 0.0);


    // ======  Update CoM by reading it from Gazebo->transforming it->Sending to ROS ====== //
    updateCoM();


    // ======  Initialization of LPV-DS Model Class ====== //
    path_model_ = path_to_models_ + lpvDS_model_name_ + "/";
    cout << "The following model will be loaded: " << path_model_ << endl;
    string path_dim    = path_model_ +  "dimensions";
    string path_Priors = path_model_ +  "Priors";
    string path_Mu     = path_model_ +  "Mu";
    string path_Sigma  = path_model_ +  "Sigma";
    string path_A      = path_model_ +  "A_k";
    
    /* Instantiate lpv-DS Model with parameters read from text files providing the path of the directory*/
    LPV_DS_.reset(new lpvDS(path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_A.c_str()));

    /* Fill in attractor from text-file*/
    
    MatrixXd attractor;
    string path_att    = path_model_ +  "attractor";
    attractor = fileUtils_.readMatrix(path_att.c_str());
    att_.resize(2);
    att_ = attractor.col(0);

    attractor_.resize(3);
    attractor_(0) = att_(0);
    attractor_(1) = att_(1);
    attractor_(2) = 0;

    // Set maximum velocity values
    max_v = 0.1;
    max_w = 0.05;
    kappa_ = 0.1;
    dist_thres_ = 0.15;    
    counter_ = 0;

    return true; 
}


void DesVelocityCommand_lpvDS::stop(){

    // Close all in/out ports
    RootlinkPose_port_In.close();
    CoMPose_port_Out.close();

}

void DesVelocityCommand_lpvDS::updateCoM(){

    RootlinkPose_values = RootlinkPose_port_In.read(); 
    Rootlink_measurements.resize(RootlinkPose_values->size());    
    for (int i= 0;i < RootlinkPose_values->size(); i++)
        Rootlink_measurements(i) = RootlinkPose_values->get(i).asDouble();            

    // Fill in local CoM Position variable
    for (int i=0; i < CoM_pos.rows(); i++)
        CoM_pos(i) = Rootlink_measurements(i);

    // Fill in local CoM Orientation variable
    for (int i=0; i < CoM_orient_rpy.rows(); i++)
        CoM_orient_rpy(i) = Rootlink_measurements(CoM_pos.rows() + i);

    std::cout << "CoM position    x: " << CoM_pos(0) << " y:" << CoM_pos(1) << " z:" << CoM_pos(2) << std::endl;

    // Create Rotation matrix from RPY angles
    Eigen::Matrix3d Rot_z, CoM_orient_robot;
    CoM_orient_robot(0,0) =  cos(CoM_orient_rpy(1))*cos(CoM_orient_rpy(2));
    CoM_orient_robot(0,1) =  sin(CoM_orient_rpy(1))*sin(CoM_orient_rpy(0))*cos(CoM_orient_rpy(2)) - sin(CoM_orient_rpy(2))*cos(CoM_orient_rpy(0));
    CoM_orient_robot(0,2) =  sin(CoM_orient_rpy(1))*cos(CoM_orient_rpy(0))*cos(CoM_orient_rpy(2)) + sin(CoM_orient_rpy(2))*sin(CoM_orient_rpy(0));

    CoM_orient_robot(1,0) =  cos(CoM_orient_rpy(1))*sin(CoM_orient_rpy(2));
    CoM_orient_robot(1,1) =  sin(CoM_orient_rpy(1))*sin(CoM_orient_rpy(0))*sin(CoM_orient_rpy(2)) + cos(CoM_orient_rpy(2))*cos(CoM_orient_rpy(0));
    CoM_orient_robot(1,2) =  sin(CoM_orient_rpy(1))*cos(CoM_orient_rpy(0))*sin(CoM_orient_rpy(2)) - cos(CoM_orient_rpy(2))*sin(CoM_orient_rpy(0));

    CoM_orient_robot(2,0) = -sin(CoM_orient_rpy(1));
    CoM_orient_robot(2,1) =  cos(CoM_orient_rpy(1))*sin(CoM_orient_rpy(0));
    CoM_orient_robot(2,2) =  cos(CoM_orient_rpy(1))*cos(CoM_orient_rpy(0));

    Rot_z << -1,   0,  0,
              0,  -1,  0,
              0,   0 , 1;

    CoM_orient_rot = CoM_orient_robot*Rot_z;

    // Convert CoM Rotation matrix to quaternion (to send to ROS)
    CoM_orient_quat = Eigen::Quaterniond(CoM_orient_rot);
    yarp::sig::Vector &output_CoMPose = CoMPose_port_Out.prepare();

    // Fill in CoMPoses vector
    CoMPoses[0] = CoM_pos(0);
    CoMPoses[1] = CoM_pos(1);
    CoMPoses[2] = CoM_pos(2);
    CoMPoses[3] = CoM_orient_quat.x();
    CoMPoses[4] = CoM_orient_quat.y();
    CoMPoses[5] = CoM_orient_quat.z();
    CoMPoses[6] = CoM_orient_quat.w();

    // Send out
    output_CoMPose = CoMPoses;
    CoMPose_port_Out.write();

}

void DesVelocityCommand_lpvDS::setAttractor(Eigen::Vector3d attractor){
    attractor_ = attractor;
}


void DesVelocityCommand_lpvDS::setkappa(double kappa){
    kappa_ = kappa;
}

Eigen::Vector3d DesVelocityCommand_lpvDS::linearDS(){
    Eigen::Vector3d x_dot;

    // x_dot = -k(x-x_att)
    x_dot = -kappa_*( CoM_pos - attractor_);
    return x_dot;
}


Eigen::Vector3d DesVelocityCommand_lpvDS::DSfromLPVDS(){
    Eigen::Vector2d x_dot_2d, att_2d (att_(0),att_(1)), x_2d (CoM_pos(0),CoM_pos(1)) ;
    x_dot_2d = LPV_DS_->compute_f(x_2d, att_2d);

    double ds_vel_limit_ (0.3);
    if (x_dot_2d.norm() > ds_vel_limit_){
        x_dot_2d = x_dot_2d / x_dot_2d.norm() * ds_vel_limit_;
    }

    std::cout << "x_dot from lpvDS class v_x:" << x_dot_2d(0) << " v_y:" << x_dot_2d(1) << std::endl;

    Eigen::Vector3d x_dot;
    x_dot(0) = x_dot_2d(0);
    x_dot(1) = x_dot_2d(1);
    x_dot(2) = 0;

    return x_dot;
}


Eigen::Vector3d DesVelocityCommand_lpvDS::RotateDS(Eigen::Vector3d x_dot){
    Eigen::Vector3d x_dot_rot;
    // Rotate velocity vector to CoM Reference frame
   std::cout << "x_dot before rotation v_x:" << x_dot(0) << " v_y:" << x_dot(1) << std::endl;
    x_dot_rot =  CoM_orient_rot.inverse() * x_dot;
//    std::cout << "x_dot after rotation v_x:" << x_dot_rot(0) << " v_y:" << x_dot_rot(1) << std::endl;

    return x_dot_rot;
}


double DesVelocityCommand_lpvDS::computeAngularVelocity(Eigen::Vector3d x_dot){
    x_dot(2) = 0;
    Eigen::Vector3d ds_dir    = x_dot.normalized();
    Eigen::Vector3d robot_dir(CoM_orient_rot(0,0),CoM_orient_rot(1,0),CoM_orient_rot(2,0));
    robot_dir = robot_dir.normalized();
    
    Eigen::Vector3d x_dir(0,0,0);
    double robot_angle_   =   atan2(x_dir(1),x_dir(0)) - atan2(robot_dir(1),robot_dir(0));
    std::cout << "Current Robot Heading Angle (wrt. world x): " << robot_angle_ << std::endl;
    std::cout << "DS direction: " << ds_dir(0) << "  " << ds_dir(1) << std::endl;

    rot_angle_  = atan2(ds_dir(1),ds_dir(0)) - atan2(robot_dir(1),robot_dir(0));
    if (rot_angle_ > M_PI) {rot_angle_ -= 2* M_PI;}
    else if (rot_angle_ <= -M_PI){rot_angle_ += 2* M_PI;}

    std::cout << "Desired Rotation Angle: " << rot_angle_ << std::endl;

    // Creating rotation matrix from current CoM to desired CoM
    Eigen::Matrix3d Rot_z;
    Rot_z << cos(rot_angle_), -sin(rot_angle_), 0,
             sin(rot_angle_),  cos(rot_angle_), 0,
                          0,               0,   1;

    // Compute angular velocity that rotates current CoM to desired CoM
    Eigen::Matrix3d CoM_orient_des, Rot_diff, Omega_x;
    CoM_orient_des = CoM_orient_rot*Rot_z;
    Rot_diff = CoM_orient_des * CoM_orient_rot.transpose();

    /* log geometric vector */
    Eigen::Matrix3d Omega;
    double theta, n_scalar;
    theta      = acos((Rot_diff.trace()-1)/2);
    n_scalar   = 1/(2*sin(theta));
    Omega(0)   =  Rot_diff(2,1) - Rot_diff(1,2);
    Omega(1)   =  Rot_diff(0,2) - Rot_diff(2,0);
    Omega(2)   =  Rot_diff(1,0) - Rot_diff(0,1);
    Omega  = theta * n_scalar * Omega;

    return Omega(2);
}

void DesVelocityCommand_lpvDS::updateDesComVel(){
                 
        updateCoM();
        attractor_(0) = att_(0);
        attractor_(1) = att_(1);
        attractor_(2) = CoM_pos(2);

        Eigen::Vector3d v_des, v_des_, tmp_;
        double w_z, dist_targ;
        std::cout << "attractor: "<< attractor_(0) << " " << attractor_(1) << " "<<  attractor_(2)<< std::endl;
        dist_targ = (CoM_pos - attractor_).norm();
        std::cout << "Distance to target: "<< dist_targ <<std::endl;

        if (dist_targ < dist_thres_){
            std::cout << "Attractor Reached!"<< std::endl;
            des_com_vel_(0) = NAN;
        }else{

            if (DSType_ == 0) {
                v_des = linearDS();
                w_z   = computeAngularVelocity(v_des);
                /* Slow down angular velocity */
                w_z   = kappa_*w_z;
            }
            else if (DSType_ == 1){

                v_des_       = DSfromLPVDS();
                v_des        = RotateDS(v_des_);
                des_omega_   = computeAngularVelocity(v_des_);
                
                if (abs(rot_angle_)> M_PI/10){ // If desired rotation angle >30deg slow down linear velocity
                    v_des(0) = 0.0*v_des(0);
                    v_des(1) = 0.0*v_des(1);
                    w_z = 0.75*des_omega_;
                }
                else
                    w_z = kappa_*des_omega_;
            }

            /* Limit linear and angular velocities */
            double desired_speed = v_des.norm();
            if (desired_speed > max_v)
                v_des = v_des / v_des.norm() * max_v;

            std::cout << "Desired Speed: " << desired_speed << " Truncated speed: " << v_des.norm() << " Desired omega:" << w_z <<  std::endl;

            if (abs(w_z) > max_w){
                if (w_z < 0)
                    w_z = -max_w;
                else
                    w_z = max_w;
            }

            /* If the robot is close to the target remove angular velocity */
            if (dist_targ < dist_thres_*3)
                w_z = 0.0;

            /* Send desired CoM velocity to walking controller */
            des_com_vel_(0) = v_des(0);
            des_com_vel_(1) = v_des(1);
            des_com_vel_(2) = w_z;

            std::cout << "Desired (CoM) Velocity DS v_x: " << des_com_vel_(0) << " v_y:" << des_com_vel_(1) << " w_z:" << des_com_vel_(2)  << std::endl;
        }

}
    
