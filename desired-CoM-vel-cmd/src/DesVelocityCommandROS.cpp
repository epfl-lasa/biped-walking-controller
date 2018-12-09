#include "DesVelocityCommandROS.h"

DesVelocityCommandROS::DesVelocityCommandROS(string moduleName, string robotName, int DSType)
                                                    : moduleName_(moduleName)
                                                    , robotName_(robotName)
                                                    , DSType_(DSType){}


bool DesVelocityCommandROS::initReader(){



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


    if (DSType_ == 1){
        // ====== Opening the port for desired DS Velocity in World ====== //
        std::string DSVelocity_portName="/";
        DSVelocity_portName += robotName_;
        DSVelocity_portName += "/DesiredCoMVelocity:o";


        DSVelocity_port_In.open("/DesiredCoMVelocity_In:i");
        if(!Network::connect(DSVelocity_portName.c_str(), DSVelocity_port_In.getName().c_str())){
            printf(" Unable to connect to DS desired velocity reader port");
            return false;
        }


        // ====== Opening the port for desired DS Attractor in World ====== //
        std::string DSAttractor_portName="/";
        DSAttractor_portName += robotName_;
        DSAttractor_portName += "/DesiredCoMAttractor:o";


        DSAttractor_port_In.open("/DesiredCoMAttractor_In:i");
        if(!Network::connect(DSAttractor_portName.c_str(), DSAttractor_port_In.getName().c_str())){
            printf(" Unable to connect to DS desired velocity reader port");
            return false;
        }

    }


    // Set maximum velocity values
    max_v = 0.15;
    max_w = 0.1;
    kappa_ = 0.15;
    dist_thres_ = 0.25;
    attractor_.setZero();

    return true; 
}


void DesVelocityCommandROS::stop(){

    // Close all in/out ports
    RootlinkPose_port_In.close();
    CoMPose_port_Out.close();

    if (DSType_ == 1){
        DSVelocity_port_In.close();
        DSAttractor_port_In.close();
    }

}


void DesVelocityCommandROS::updateCoM(){

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
//    std::cout << "CoM orientation roll: " << CoM_orient_rpy(0) << " pitch:" << CoM_orient_rpy(1) << " yaw:" << CoM_orient_rpy(2) <<  std::endl;

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

void DesVelocityCommandROS::setAttractor(Eigen::Vector3d attractor){
    attractor_ = attractor;
//    std::cout << "Desired CoM Target x: " << attractor_(0) << " y:" << attractor_(1) << " z:" << attractor_(2) << std::endl;
}


void DesVelocityCommandROS::setkappa(double kappa){
    kappa_ = kappa;
//    std::cout << "Desired kappa: " << kappa_ << std::endl;
}

Eigen::Vector3d DesVelocityCommandROS::linearDS(){
    Eigen::Vector3d x_dot;

    // x_dot = -k(x-x_att)
    x_dot = -kappa_*( CoM_pos - attractor_);
    return x_dot;
}


Eigen::Vector3d DesVelocityCommandROS::DSfromROS(){

    // Update attractor
    DSAttractor_values = DSAttractor_port_In.read();
    Eigen::Vector3d updated_attractor (DSAttractor_values->get(0).asDouble(), DSAttractor_values->get(1).asDouble(), CoM_pos(2));
    setAttractor(updated_attractor);

    // Update desired Velocity from DS
    DSVelocity_values = DSVelocity_port_In.read();
    Eigen::Vector3d x_dot;
    for (int i= 0; i < 3; i++)
        x_dot(i) = DSVelocity_values->get(i).asDouble();

    return x_dot;
}

double DesVelocityCommandROS::computeAngularVelocity(Eigen::Vector3d x_dot){

    Eigen::Vector3d ds_dir    = x_dot.normalized();
    Eigen::Vector3d robot_dir(CoM_orient_rot(0,0),CoM_orient_rot(1,0),CoM_orient_rot(2,0));
    robot_dir = robot_dir.normalized();
    rot_angle_  = atan2(ds_dir(1),ds_dir(0)) - atan2(robot_dir(1),robot_dir(0));
    std::cout << "Desired Rotation Angle: " << rot_angle_ << std::endl;

    // Creating rotation matrix from current CoM to desired CoM
    Eigen::Matrix3d Rot_z;
    Rot_z << cos(rot_angle_), -sin(rot_angle_), 0,
             sin(rot_angle_),  cos(rot_angle_), 0,
                          0,               0, 1;

    // Compute angular velocity that rotates current CoM to desired CoM
    Eigen::Matrix3d CoM_orient_des, Rot_diff, Omega_x;
    CoM_orient_des = CoM_orient_rot*Rot_z;
    Rot_diff = CoM_orient_des * CoM_orient_rot.transpose();
    Omega_x = Rot_diff.log();

    Eigen::Vector3d ang_vel (Omega_x(2,1),Omega_x(0,2),Omega_x(1,0));
    return ang_vel(2);
}

void DesVelocityCommandROS::updateDesComVel(){
                 

        updateCoM();

        Eigen::Vector3d v_des;
        double w_z, dist_targ;
        dist_targ = (CoM_pos-attractor_).norm();
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
                v_des = DSfromROS();
                w_z   = computeAngularVelocity(v_des);
            }

            /* Limit linear and angular velocities */
            double desired_speed = v_des.norm();
            if (desired_speed > max_v)
                v_des = v_des / v_des.norm() * max_v;

            std::cout << "Desired Speed: " << desired_speed << " Truncated speed: " << v_des.norm() <<  std::endl;

            if (abs(w_z) > max_w){
                if (w_z < 0)
                    w_z = -max_w;
                else
                    w_z = max_w;
            }

            /* Stop linear velocity if desired rotation angle is too high */
            if (abs(rot_angle_)> M_PI/6){
              v_des(0) = 0;
              v_des(1) = 0;
              w_z  = 2*kappa_*w_z;
            }
            else{
                /* Scale angular velocity to avoid falling */
                w_z = kappa_*w_z;
            }

            /* Send desired CoM velocity to walking controller */
            des_com_vel_(0) = v_des(0);
            des_com_vel_(1) = v_des(1);
            des_com_vel_(2) = w_z;
            std::cout << "Desired (CoM) Velocity DS v_x: " << des_com_vel_(0) << " v_y:" << des_com_vel_(1) << " w_z:" << des_com_vel_(2)  << std::endl;
        }

}
    