#include "DesVelocityReader.h"

DesVelocityReader::DesVelocityReader(string moduleName, string robotName, int VelocityCmdType, Eigen::Vector3d  init_vel)
                                                    : moduleName_(moduleName)
                                                    , robotName_(robotName)
                                                    , VelocityCmdType_(VelocityCmdType)
                                                    , init_vel_(init_vel){}


bool DesVelocityReader::initReader(){


    // ====== Opening the port for the Keyboard input cmds w/moduleName ====== //
    std::string KeyboardOutputs_port ="/";
                KeyboardOutputs_port += moduleName_;
                KeyboardOutputs_port += "/keyboardOutputs:o";


    KeyboardCmd_port_In.open("/KeyboardInputCmds:i");

    if(!Network::connect(KeyboardOutputs_port.c_str(), KeyboardCmd_port_In.getName().c_str())){
        printf(" Unable to connect to the KeyboardCmdsReaderModule port");
        return false;
    }


    // ====== Opening the port for Root-Link in World (CoM) Pose reader w/robotName ====== //

    std::string RootlinkPose_portName="/";
    RootlinkPose_portName += robotName_;
    RootlinkPose_portName += "/get_root_link_WorldPose:o";


    RootlinkPose_port_In.open("/RootlinkPose_In:i");
    if(!Network::connect(RootlinkPose_portName.c_str(), RootlinkPose_port_In.getName().c_str())){
        printf(" Unable to connect to the KeyboardCmdsReaderModule port");
        return false;
    }

    std::cout << " ================== Initial ROTATION!!!! ==================="<< std::endl;
    updateCoM();
    std::cout << "Initial Velocity vx:" << init_vel_(0) << " vy:" << init_vel_(1) <<  " wz:" << init_vel_(2) << std::endl;
    std::cout << " ================== Initial ROTATION!!!! ==================="<< std::endl;

    des_com_vel_.resize(3);
    des_com_vel_.setZero();


    // ======  Filling in Initial Velocity Vector====== //
    if(VelocityCmdType_ < 2){
        
        keyboardValues  = KeyboardCmd_port_In.read(); 
        incr_vel_.resize(keyboardValues->size());
        incr_vel_.setZero();

        std::cout << "Initial Velocity vx:" << init_vel_(0) << " vy:" << init_vel_(1) <<  " wz:" << init_vel_(2) << std::endl;
        des_com_vel_(0) = init_vel_(0);
        des_com_vel_(1) = init_vel_(1);
        des_com_vel_(2) = init_vel_(2);           
    }


    // Set maximum velocity values
    max_v = 0.2;
    max_w = 0.2;

    return true; 
}


void DesVelocityReader::stop(){

    KeyboardCmd_port_In.close();
    return ;
}


void DesVelocityReader::updateCoM(){

    RootlinkPose_values = RootlinkPose_port_In.read(); 
    Rootlink_measurements.resize(RootlinkPose_values->size());    
    for (int i= 0;i < RootlinkPose_values->size(); i++){
        Rootlink_measurements(i) = RootlinkPose_values->get(i).asDouble();            
    }

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
//    std:: cout << "Aligned CoM Rotation matrix "<< std::endl << CoM_orient_rot << std::endl;

}

void DesVelocityReader::setAttractor(Eigen::Vector3d attractor){
    attractor_ = attractor;
    std::cout << "Desired Root-Link (CoM) Target x: " << attractor_(0) << " y:" << attractor_(1) << " z:" << attractor_(2) << std::endl;
}


// This can be a separate class of sets of different DS; e.g. linear, non-linear, lags
Eigen::Vector3d DesVelocityReader::linearDS(double kappa){
    Eigen::Vector3d x_dot;
    Eigen::Vector3d x_com;
    for(int i=0; i<x_com.size(); i++)
        x_com(i) =  Rootlink_measurements(i);

    // x_dot = -k(x-x_att)
    x_dot = -kappa*( x_com - attractor_);
    return x_dot;
}

double DesVelocityReader::computeAngularVelocity(Eigen::Vector3d x_dot){

    Eigen::Vector3d ds_dir    = x_dot.normalized();
    Eigen::Vector3d robot_dir(CoM_orient_rot(0,0),CoM_orient_rot(1,0),CoM_orient_rot(2,0));
    double rot_angle  = acos(ds_dir.transpose()*robot_dir);

    // Creating rotation matrix from current CoM to desired CoM
    Eigen::Matrix3d Rot_z;
    Rot_z << cos(rot_angle), -sin(rot_angle), 0,
            sin(rot_angle),  cos(rot_angle), 0,
                          0,               0, 1;

    // Compute angular velocity that rotates current CoM to desired CoM
    Eigen::Matrix3d CoM_orient_des, Rot_diff, Omega_x;
    CoM_orient_des = CoM_orient_rot*Rot_z;
    Rot_diff = CoM_orient_des * CoM_orient_rot.transpose();
    Omega_x = Rot_diff.log();

    Eigen::Vector3d ang_vel (Omega_x(2,1),Omega_x(0,2),Omega_x(1,0));
//    std::cout << "Angular Velocity w_x:" << ang_vel(0) << " w_y:" << ang_vel(1) << " w_z:" << ang_vel(2);

    return ang_vel(2);
}

void DesVelocityReader::updateDesComVel(){
                 

    if (VelocityCmdType_ < 2){
        // Extract the read value
        keyboardValues  = KeyboardCmd_port_In.read();
        incr_vel_(0)    = keyboardValues->get(0).asDouble();
        incr_vel_(1)    = keyboardValues->get(1).asDouble();
        incr_vel_(2)    = keyboardValues->get(2).asDouble();

        // This is a stopping condition
        if (isnan(incr_vel_(0)))
            des_com_vel_(0) = NAN;
        else{
                if(VelocityCmdType_ == 1) {
                    // Add increments
                    des_com_vel_(0) = des_com_vel_(0) + incr_vel_(0);
                    des_com_vel_(1) = des_com_vel_(1) + incr_vel_(1);
                    des_com_vel_(2) = des_com_vel_(2) + incr_vel_(2);

                    // Truncate velocities
                    // forward and backward walking
                    // -----------------------------
                    if(des_com_vel_(0) >= max_v && incr_vel_(0) > 0.0) { des_com_vel_(0) = max_v; }
                    if(des_com_vel_(0) <= -max_v && incr_vel_(0) < 0.0) {  des_com_vel_(0) = -max_v;  }


                    // lateral walking
                    // ---------------
                    if(des_com_vel_(1) >= max_v && incr_vel_(1) > 0.0) { des_com_vel_(1) = max_v; }
                    if(des_com_vel_(1) <= -max_v && incr_vel_(1) < 0.0) {  des_com_vel_(1) = -max_v;  }

                    // rotation
                    // --------
                    if(des_com_vel_(2) >= max_w && incr_vel_(2) > 0.0) { des_com_vel_(2) = max_w; }
                    if(des_com_vel_(2) <= -max_w && incr_vel_(2) < 0.0) {  des_com_vel_(2) = -max_w; }

                }
                else if(VelocityCmdType_ == 0) {

                    // Fix COM velocity using value from .ini config file
                    // # Initial velocity Vx [m/s], Vy [m/s],  Wz [rad/s],
                    des_com_vel_(0) = init_vel_(0);
                    des_com_vel_(1) = init_vel_(0);
                    des_com_vel_(2) = init_vel_(0);
                }
        }

    }else{
        // For DS Velocity Command Type (2)
        updateCoM();

        double kappa = 0.1;
        Eigen::Vector3d v_des;
        double w_z;

        double dist_targ = (CoM_pos-attractor_).norm();
        std::cout << "Distance to target: "<< dist_targ <<std::endl;

        if (dist_targ < 0.15){
            std::cout << "Attractor Reached!"<< std::endl;
            des_com_vel_(0) = NAN;
        }else{
            v_des = linearDS(kappa);
            w_z   = computeAngularVelocity(v_des);
            std::cout << "Desired (CoM) Velocity DS v_x: " << v_des(0) << " v_y:" << v_des(1) << " w_z:" << w_z << std::endl;

            // Slow down angular velocity
            w_z   = kappa*computeAngularVelocity(v_des);

            // Truncate values with maximum velocity limits
            if(v_des(0) > max_v)
                des_com_vel_(0) = max_v;
            else if (v_des(0) < -max_v)
                des_com_vel_(0) = -max_v;
            else
                des_com_vel_(0) = v_des(0);

            if(v_des(1) > max_v)
                des_com_vel_(1) = max_v;
            else if (v_des(1) < -max_v)
                des_com_vel_(1) = -max_v;
            else
                des_com_vel_(1) = v_des(1);

            if (w_z > max_w)
                des_com_vel_(2) = max_w;
            else if(w_z < -max_w)
                des_com_vel_(2) = -max_w;
            else
                des_com_vel_(2) = w_z;


        }

    }

}
    

