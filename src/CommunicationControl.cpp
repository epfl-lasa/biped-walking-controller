
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


// ===============================================================================================
// ControlledDevices : This class comprises 4 subclasses that ensure communication and control 
// CommunicationControl
// 	7.1. ControlledDevices
// 	7.2. RobotSensors
// 	7.3. RobotPostures
// 	7.4. SyncRobotModelStanceFoot
// ===============================================================================================

#include "CommunicationControl.h"


// ===============================================================================================

// ===============================================================================================

ControlledDevices::ControlledDevices() {}
        
ControlledDevices::~ControlledDevices()
{
    ControlledDevices::CloseLegsDevices();
    ControlledDevices::CloseArmsDevices();
    ControlledDevices::CloseTorsoDevices();
}

void ControlledDevices::CreateRobotDevices(std::string robotName,
                                           std::string moduleName)
{
    RobotName  = robotName;
    ModuleName = moduleName;
}

void ControlledDevices::OpenLegsDevices()
{
    // local and remote port
    // left
    std::string localPorts_lleg = "/";
    localPorts_lleg += ModuleName;
    localPorts_lleg += "/left_leg";

    std::string remotePorts_lleg="/";
    remotePorts_lleg+=RobotName;
    remotePorts_lleg+="/left_leg";

    // right
    std::string localPorts_rleg = "/";
    localPorts_rleg += ModuleName;
    localPorts_rleg += "/right_leg";

    std::string remotePorts_rleg="/";
    remotePorts_rleg+=RobotName;
    remotePorts_rleg+="/right_leg";

    // Devices property
    Property optionsLLeg;
    optionsLLeg.put("device", "remote_controlboard");
    optionsLLeg.put("local", localPorts_lleg.c_str());
    optionsLLeg.put("remote", remotePorts_lleg.c_str());


    Property optionsRLeg;
    optionsRLeg.put("device", "remote_controlboard");
    optionsRLeg.put("local", localPorts_rleg.c_str());
    optionsRLeg.put("remote", remotePorts_rleg.c_str());


    clientLeftLeg.open(optionsLLeg);
    if (!clientLeftLeg.isValid()) {
        printf("left leg Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }

    clientLeftLeg.view(iencs_left_leg);
    clientLeftLeg.view(ipos_left_leg);


    clientRightLeg.open(optionsRLeg);
    if (!clientRightLeg.isValid()) {
        printf("right leg Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }

           
    bool ok;
         ok = clientLeftLeg.view(ipos_left_leg);
         ok = ok && clientLeftLeg.view(iencs_left_leg);
         ok = ok && clientLeftLeg.view(iimp_left_leg);
         ok = ok && clientLeftLeg.view(ictrl_left_leg);
         ok = ok && clientLeftLeg.view(iint_left_leg);
         ok = ok && clientLeftLeg.view(ipos_left_legDir);
         ok = ok && clientLeftLeg.view(itrq_left_leg);
         ok = ok && clientLeftLeg.view(ivel_left_leg); 


         ok = ok && clientRightLeg.view(ipos_right_leg);
         ok = ok && clientRightLeg.view(iencs_right_leg);
         ok = ok && clientRightLeg.view(iimp_right_leg);
         ok = ok && clientRightLeg.view(ictrl_right_leg);
         ok = ok && clientRightLeg.view(iint_right_leg);
         ok = ok && clientRightLeg.view(ipos_right_legDir);
         ok = ok && clientRightLeg.view(itrq_right_leg);
         ok = ok && clientRightLeg.view(ivel_right_leg);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        //return 0;
    }
       
        iencs_left_leg->getAxes(&lljoints);         
        iencs_right_leg->getAxes(&rljoints);   

        // resize the commands and the encoders vectors
         // legs 
        // resize the vector of encoders and commands values
        encoders_left_leg.resize(lljoints);
        encoders_right_leg.resize(rljoints);

        commands_left_leg.resize(lljoints);
        commands_right_leg.resize(rljoints);    

        // get the initial encoders values
        printf("waiting for encoders");
        while(!this->iencs_left_leg->getEncoders(this->encoders_left_leg.data()) &&
              !this->iencs_right_leg->getEncoders(this->encoders_right_leg.data()))
        {
            Time::delay(0.1);
            printf(".");
        }  
            printf("\n;"); 

        //
        this->commands_left_leg  = this->encoders_left_leg;
        this->commands_right_leg = this->encoders_right_leg;

}


void ControlledDevices::setLegsTrajectoryParameters( double AccelParam,
                                                     double SpeedParam)
{
    yarp::sig::Vector tmp;
    tmp.resize(lljoints);

    // left
    for (int i = 0; i < lljoints; i++) {
        tmp[i] = AccelParam;
    }
        ipos_left_leg->setRefAccelerations(tmp.data());

    for (int i = 0; i < lljoints; i++) {
        tmp[i] = SpeedParam;
        ipos_left_leg->setRefSpeed(i, tmp[i]);
    }

    // right
    for (int i = 0; i < rljoints; i++) {
        tmp[i] = AccelParam;
    }
        ipos_right_leg->setRefAccelerations(tmp.data());

    for (int i = 0; i < rljoints; i++) {
        tmp[i] = SpeedParam;
        ipos_right_leg->setRefSpeed(i, tmp[i]);
    }

}



void ControlledDevices::CloseLegsDevices()
{
    ipos_left_leg->stop();
    clientLeftLeg.close();

    ipos_right_leg->stop();
    clientRightLeg.close();       
}

// arms
// ~~~~~

void ControlledDevices::OpenArmsDevices()
{
    // local and remote port
    // left
    std::string localPorts_larm = "/";
    localPorts_larm += ModuleName;
    localPorts_larm += "/left_arm";

    std::string remotePorts_larm="/";
    remotePorts_larm += RobotName;
    remotePorts_larm += "/left_arm";

    // right
    std::string localPorts_rarm = "/";
    localPorts_rarm += ModuleName;
    localPorts_rarm += "/right_arm";

    std::string remotePorts_rarm="/";
    remotePorts_rarm += RobotName;
    remotePorts_rarm += "/right_arm";

    // Devices property
    Property optionsLArm;
    optionsLArm.put("device", "remote_controlboard");
    optionsLArm.put("local", localPorts_larm.c_str());
    optionsLArm.put("remote", remotePorts_larm.c_str());

    Property optionsRArm;
    optionsRArm.put("device", "remote_controlboard");
    optionsRArm.put("local", localPorts_rarm.c_str());
    optionsRArm.put("remote", remotePorts_rarm.c_str());

    clientLeftArm.open(optionsLArm);
    if (!clientLeftArm.isValid()) {
        printf("left Arm Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }

    clientLeftArm.view(iencs_left_arm);
    clientLeftArm.view(ipos_left_arm);


    clientRightArm.open(optionsRArm);
    if (!clientRightArm.isValid()) {
        printf("right arm Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }
         
    bool ok;
         ok = clientLeftArm.view(ipos_left_arm);
         ok = ok && clientLeftArm.view(iencs_left_arm);
         ok = ok && clientLeftArm.view(iimp_left_arm);
         ok = ok && clientLeftArm.view(ictrl_left_arm);
         ok = ok && clientLeftArm.view(iint_left_arm);
         ok = ok && clientLeftArm.view(ipos_left_armDir);
         ok = ok && clientLeftArm.view(itrq_left_arm);
         ok = ok && clientLeftArm.view(ivel_left_arm);



         ok = ok && clientRightArm.view(ipos_right_arm);
         ok = ok && clientRightArm.view(iencs_right_arm);
         ok = ok && clientRightArm.view(iimp_right_arm);
         ok = ok && clientRightArm.view(ictrl_right_arm);
         ok = ok && clientRightArm.view(iint_right_arm);
         ok = ok && clientRightArm.view(ipos_right_armDir);
         ok = ok && clientRightArm.view(itrq_right_arm);
         ok = ok && clientRightArm.view(ivel_right_arm);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        //return 0;
    }
          
    iencs_left_arm->getAxes(&lajoints);         
    iencs_right_arm->getAxes(&rajoints);

    // resize the vector of encoders and commands values
    encoders_left_arm.resize(lajoints);
    encoders_right_arm.resize(rajoints);

    commands_left_arm.resize(lajoints);
    torqueCmd_left_arm.resize(lajoints);
    commands_right_arm.resize(rajoints);
    torqueCmd_right_arm.resize(lajoints);
    VelocityCmd_left_arm.resize(lajoints);
    VelocityCmd_right_arm.resize(rajoints);
    VelocityCmd_left_arm  = 100.0;
    VelocityCmd_right_arm = 100.0;

    printf("waiting for encoders");
    while(!this->iencs_left_arm->getEncoders(this->encoders_left_arm.data()) &&
          !this->iencs_right_arm->getEncoders(this->encoders_right_arm.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
        printf("\n;");

    //
    this->commands_left_arm  = this->encoders_left_arm;
    this->commands_right_arm = this->encoders_right_arm;

    this->ivel_left_arm->setRefAccelerations(VelocityCmd_left_arm.data());
    this->ivel_right_arm->setRefAccelerations(VelocityCmd_right_arm.data());

    VelocityCmd_left_arm  = 0.0;
    VelocityCmd_right_arm = 0.0;
                
}

void ControlledDevices::setArmsTrajectoryParameters( double AccelParam,
                                                     double SpeedParam)
{
    yarp::sig::Vector tmp;
    tmp.resize(lajoints);
    
    // left
    for (int i = 0; i < lajoints; i++) {
        tmp[i] = AccelParam;
    }
        ipos_left_arm->setRefAccelerations(tmp.data());

    for (int i = 0; i < lajoints; i++) {
        tmp[i] = SpeedParam;
        ipos_left_arm->setRefSpeed(i, tmp[i]);
    }

    // right
    for (int i = 0; i < rajoints; i++) {
        tmp[i] = AccelParam;
    }
        ipos_right_arm->setRefAccelerations(tmp.data());

    for (int i = 0; i < rajoints; i++) {
        tmp[i] = SpeedParam;
        ipos_right_arm->setRefSpeed(i, tmp[i]);
    }

}

void ControlledDevices::CloseArmsDevices()
{
    ipos_left_arm->stop();
    clientLeftArm.close();

    ipos_right_arm->stop();
    clientRightArm.close();       
}

// Torso
// ~~~~~
        
void ControlledDevices::OpenTorsoDevices()
{
    // local and remote port
        // left
    std::string localPorts_torso = "/";
    localPorts_torso += ModuleName;
    localPorts_torso += "/torso";

    std::string remotePorts_torso = "/";
    remotePorts_torso += RobotName;
    remotePorts_torso += "/torso";

    // Devices property
    Property optionsTorso;
    optionsTorso.put("device", "remote_controlboard");
    optionsTorso.put("local", localPorts_torso.c_str());
    optionsTorso.put("remote", remotePorts_torso.c_str());

    clientTorso.open(optionsTorso);
    if (!clientTorso.isValid()) {
        printf("torso Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }
          
    bool ok;
         ok = clientTorso.view(ipos_torso);
         ok = ok && clientTorso.view(iencs_torso);
         ok = ok && clientTorso.view(iimp_torso);
         ok = ok && clientTorso.view(ictrl_torso);
         ok = ok && clientTorso.view(iint_torso);
         ok = ok && clientTorso.view(ipos_torsoDir);
         ok = ok && clientTorso.view(itrq_torso);

    if (!ok) 
    {
        printf("Problems acquiring interfaces\n");
                   //return 0;
        }

        iencs_torso->getAxes(&torsojoints); 

        // resize the vector of encoders and commands values
        encoders_torso.resize(torsojoints);
        commands_torso.resize(torsojoints);    

        printf("waiting for torso encoders");
        while(!this->iencs_torso->getEncoders(this->encoders_torso.data()))
        {
            Time::delay(0.1);
            printf(".");
        }
            printf("\n;");      

        this->commands_torso  = this->encoders_torso;     
}

void ControlledDevices::setTorsoTrajectoryParameters( double AccelParam,
                                                      double SpeedParam)
{
    yarp::sig::Vector tmp;
    tmp.resize(torsojoints);
    
    // left
    for (int i = 0; i < torsojoints; i++) {
        tmp[i] = AccelParam;
    }
        ipos_torso->setRefAccelerations(tmp.data());

    for (int i = 0; i < torsojoints; i++) {
        tmp[i] = SpeedParam;
        ipos_torso->setRefSpeed(i, tmp[i]);
    }

}

void ControlledDevices::CloseTorsoDevices()
{
    ipos_torso->stop();
    clientTorso.close();      
}

// ===============================================================================================

// ===============================================================================================

RobotSensors::RobotSensors() {};

RobotSensors::~ RobotSensors()
{
  	RobotSensors::CloseRobotSensors();

    if (Filter_FT_LeftArm) {
        delete Filter_FT_LeftArm;
        Filter_FT_LeftArm = 0;
    }

    if (Filter_FT_RightArm) {
        delete Filter_FT_RightArm;
        Filter_FT_RightArm = 0;
    }

}

void RobotSensors::OpenRobotSensors(std::string robotName, ControlledDevices &botDevices, double SamplingTime)
{
  	// Opening CoM and IMU ports
  	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
   	// CoM_port_In.open("/CoM_In:i");
    // Network::connect("/wholeBodyDynamicsTree/com:o",CoM_port_In.getName().c_str());

    IMU_port_In.open("/IMU_In:i");
    std::string imu_portName="/";
	            imu_portName += robotName;
	            imu_portName += "/inertial";         
    Network::connect(imu_portName.c_str(), IMU_port_In.getName().c_str());

    // Opening force/torque sensors ports
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    	// left
    l_foot_FT_inputPort.open("/l_foot_FT_readPort");
    std::string l_foot_FT_portName ="/";
	            l_foot_FT_portName += robotName;
	            l_foot_FT_portName += "/left_foot/analog:o";
    Network::connect(l_foot_FT_portName.c_str(),l_foot_FT_inputPort.getName().c_str());
       // right
    r_foot_FT_inputPort.open("/r_foot_FT_readPort");
    std::string r_foot_FT_portName="/";
	            r_foot_FT_portName += robotName;
	            r_foot_FT_portName += "/right_foot/analog:o";
    Network::connect(r_foot_FT_portName.c_str(),r_foot_FT_inputPort.getName().c_str());

    // Opening force/torque sensors ports
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      	// left
    l_arm_FT_inputPort.open("/l_arm_FT_readPort");
    std::string l_arm_FT_portName="/";
	            l_arm_FT_portName += robotName;
	            l_arm_FT_portName += "/left_arm/analog:o";
    Network::connect(l_arm_FT_portName.c_str(),l_arm_FT_inputPort.getName().c_str());
       	// right
    r_arm_FT_inputPort.open("/r_arm_FT_readPort");
    std::string r_arm_FT_portName="/";
	            r_arm_FT_portName += robotName;
	            r_arm_FT_portName += "/right_arm/analog:o";
    Network::connect(r_arm_FT_portName.c_str(),r_arm_FT_inputPort.getName().c_str());

    //
    Time::delay(0.02);
    // Reading of all created ports
    // CoM_values     = CoM_port_In.read(); 
    IMU_values     = IMU_port_In.read();  
    l_arm_FT_data  = l_arm_FT_inputPort.read();
    r_arm_FT_data  = r_arm_FT_inputPort.read();   
    l_foot_FT_data = l_foot_FT_inputPort.read();
    r_foot_FT_data = r_foot_FT_inputPort.read();   
    //
    // Resizing of the measurement vectors    
    // CoM_measurements.resize(CoM_values->size()); 
    Inertial_measurements.resize(IMU_values->size());
	m_acceleration.resize(3);
	m_orientation_rpy.resize(3, 0.0);
	m_gyro_xyz.resize(3);

	l_arm_FT_vector.resize(l_arm_FT_data->size());
    r_arm_FT_vector.resize(r_arm_FT_data->size());

    l_foot_FT_vector.resize(l_foot_FT_data->size());
    r_foot_FT_vector.resize(r_foot_FT_data->size());


    // force torque sensor offset
    Arms_ForceTorqueOffset.setZero();



    // initalize the low pass filter for the force torque measurement
    Filter_FT_LeftArm   = new firstOrderIntegrator(SamplingTime, 2., 2., l_arm_FT_vector);
    Filter_FT_RightArm  = new firstOrderIntegrator(SamplingTime, 2., 2., r_arm_FT_vector);

    // getting the force torque sensor offset 
    RobotSensors::get_ArmsForceTorqueOffsets();
    

}

void RobotSensors::get_ArmsForceTorqueOffsets()
{
    // getting the force torque sensor offset
    Arms_ForceTorqueOffset.setZero();

    int nb_meas = 20;
    for (int j=0; j< nb_meas; j++)
    {
        l_arm_FT_data  = l_arm_FT_inputPort.read();
        r_arm_FT_data  = r_arm_FT_inputPort.read();

        for (int i=0; i<l_arm_FT_data->size(); i++) {
            l_arm_FT_vector(i)= l_arm_FT_data->get(i).asDouble();
        }
        for (int i=0; i<r_arm_FT_data->size(); i++) {
            r_arm_FT_vector(i)= r_arm_FT_data->get(i).asDouble();
        }

        l_arm_FT_vector = Filter_FT_LeftArm->getRK4Integral(l_arm_FT_vector);
        r_arm_FT_vector = Filter_FT_RightArm->getRK4Integral(r_arm_FT_vector);

        Arms_ForceTorqueOffset(0) += 1./nb_meas * (l_arm_FT_vector(0)+r_arm_FT_vector(0));
        Arms_ForceTorqueOffset(1) += 1./nb_meas * (l_arm_FT_vector(1)+r_arm_FT_vector(1));

        Time::delay(0.01);

    }
}

// method that reads the left leg joint encoder values and returns them
yarp::sig::Vector RobotSensors::getLeftLegEncodersValue(ControlledDevices &botDevices)
{
    botDevices.iencs_left_leg->getEncoders(botDevices.encoders_left_leg.data());

    return botDevices.encoders_left_leg;
}

// method that reads the right leg joint encoder values and returns them
yarp::sig::Vector RobotSensors::getRightLegEncodersValue(ControlledDevices &botDevices)
{
    botDevices.iencs_right_leg->getEncoders(botDevices.encoders_right_leg.data());

    return botDevices.encoders_right_leg;
}

// method that reads the left arm joint encoder values and returns them
yarp::sig::Vector RobotSensors::getLeftArmEncodersValue(ControlledDevices &botDevices)
{
    botDevices.iencs_left_arm->getEncoders(botDevices.encoders_left_arm.data());

    return botDevices.encoders_left_arm;
}

// method that reads the right arm joint encoder values and returns them
yarp::sig::Vector RobotSensors::getRightArmEncodersValue(ControlledDevices &botDevices)
{
    botDevices.iencs_right_arm->getEncoders(botDevices.encoders_right_arm.data());

    return botDevices.encoders_right_arm;
}

// method that reads the torso joint encoder values and returns them
yarp::sig::Vector RobotSensors::getTorsoEncodersValue(ControlledDevices &botDevices)
{
    botDevices.iencs_torso->getEncoders(botDevices.encoders_torso.data());

    return botDevices.encoders_torso;
}

// close the robot sensor class
void RobotSensors::CloseRobotSensors()
{
   	IMU_port_In.close();
    // CoM_port_In.close();

    l_foot_FT_inputPort.close();
    r_foot_FT_inputPort.close();

    l_arm_FT_inputPort.close();
    r_arm_FT_inputPort.close();
}

// Eigen::VectorXd RobotSensors::getCoMValues()
// {
// 	// Reading of CoM measurement
//     CoM_values  = CoM_port_In.read();        
//     //   
//     for (int i=0; i<CoM_values->size(); i++)
//     {
//         CoM_measurements(i)= CoM_values->get(i).asDouble();
//     }

//     return CoM_measurements;
// }

yarp::sig::Vector RobotSensors::getImuOrientationValues()
{
	// Reading of inertial measurement
    IMU_values  = IMU_port_In.read();

    for (int i=0; i<IMU_values->size(); i++) {
        Inertial_measurements(i)= IMU_values->get(i).asDouble();
    }
    // roll pitch yaw
    m_orientation_rpy[0] = IMU_values->get(0).asDouble();
    m_orientation_rpy[1] = IMU_values->get(1).asDouble();
    m_orientation_rpy[2] = IMU_values->get(2).asDouble();

    return m_orientation_rpy;

}

Eigen::VectorXd RobotSensors::getImuAccelerometerValues()
{
	IMU_values  	  = IMU_port_In.read();
	// Resize
    //Inertial_measurements.resize(IMU_values->size());
    for (int i=0; i<IMU_values->size(); i++) {
        Inertial_measurements(i)= IMU_values->get(i).asDouble();
    }
    // accelerations 
    m_acceleration(0) = IMU_values->get(3).asDouble();
    m_acceleration(1) = IMU_values->get(4).asDouble();
    m_acceleration(2) = IMU_values->get(5).asDouble();
	
	return m_acceleration;
}

Eigen::VectorXd RobotSensors::getImuGyroValues()
{
	IMU_values    = IMU_port_In.read();
    m_gyro_xyz(0) = IMU_values->get(6).asDouble();
    m_gyro_xyz(1) = IMU_values->get(7).asDouble();
    m_gyro_xyz(2) = IMU_values->get(8).asDouble();
	
	return m_gyro_xyz;
}

Eigen::VectorXd RobotSensors::getLeftArmForceTorqueValues()
{
	l_arm_FT_data  = l_arm_FT_inputPort.read();

    for (int i=0; i<l_arm_FT_data->size(); i++) {
        l_arm_FT_vector(i)= l_arm_FT_data->get(i).asDouble();
    }
    return l_arm_FT_vector;
}

Eigen::VectorXd RobotSensors::getRightArmForceTorqueValues()
{
	r_arm_FT_data  = r_arm_FT_inputPort.read();

    for (int i=0; i<r_arm_FT_data->size(); i++) {
        r_arm_FT_vector(i)= r_arm_FT_data->get(i).asDouble();
    }
    return r_arm_FT_vector;
}

Eigen::VectorXd RobotSensors::getLeftLegForceTorqueValues()
{
	l_foot_FT_data  = l_foot_FT_inputPort.read(); 

    for (int i=0; i<l_foot_FT_data->size(); i++) {
        l_foot_FT_vector(i)= l_foot_FT_data->get(i).asDouble();
    }
    return l_foot_FT_vector;
}

Eigen::VectorXd RobotSensors::getRightLegForceTorqueValues()
{
	r_foot_FT_data  = r_foot_FT_inputPort.read();

    for (int i=0; i<r_foot_FT_data->size(); i++) {
        r_foot_FT_vector(i)= r_foot_FT_data->get(i).asDouble();
    }
    return r_foot_FT_vector;
}

// // ===============================================================================================

// // ===============================================================================================

RobotPostures::RobotPostures(){}

RobotPostures::~RobotPostures(){}

void RobotPostures::moveToLegsPosture(       IEncoders *iencs_left,
                                             IEncoders *iencs_right,
                                      IPositionControl *ipos_left,
                                      IPositionControl *ipos_right,
                                                Vector JointsOffset,
                                     yarp::sig::Vector Des_ljoints,
                                     yarp::sig::Vector Des_rjoints)
{

    int ljoints, rjoints;
    iencs_left->getAxes(&ljoints);
    iencs_right->getAxes(&rjoints);

    // create vetors to contain the encoders and commands values
    yarp::sig::Vector  encoders_left, commands_left;
    yarp::sig::Vector  encoders_right, commands_right;
        // left
    encoders_left.resize(ljoints);
    commands_left.resize(ljoints);
        // right
    encoders_right.resize(rjoints);
    commands_right.resize(rjoints);


    yarp::sig::Vector Desq_l, Desq_r;
    Desq_l.resize(ljoints);
    Desq_r.resize(rjoints);

    Desq_l = Des_ljoints;
    Desq_r = Des_rjoints;

    Desq_l += JointsOffset;
    Desq_r += JointsOffset;

    Vector Delta_q_l, Delta_q_r;
    Delta_q_l.resize(ljoints);
    Delta_q_r.resize(rjoints);

    // 
    double DeltaJoint_l_0, 
           DeltaJoint_r_0;

    double DeltaJoint_l_1, 
           DeltaJoint_r_1;

    int nb_iter = 30;
    double alpha;
           alpha = 0;


    for (int ii=0; ii<2; ii++)
    {
            // getting actual joints values
            while(!iencs_left->getEncoders(encoders_left.data()))
            {
                  Time::delay(0.02);
                  printf(".");
            };
            while(!iencs_right->getEncoders(encoders_right.data()))
            {
                  Time::delay(0.02);
                  printf(".");
            };

            // compute the joints position errors
            for (int k=0; k<ljoints; k++)
            {
                Delta_q_l[k] = Desq_l[k] - encoders_left[k];
                Delta_q_r[k] = Desq_r[k] - encoders_right[k];
            }


        for(int i=0; i<nb_iter+1; i++)
        {
            alpha = 3.*pow(i,2.)/pow(nb_iter, 2.)-2.*pow(i,3.)/pow(nb_iter, 3.);

            switch (ii)
                {
                    case 0: 
                    {
                            DeltaJoint_l_0 = Delta_q_l[0];
                            DeltaJoint_l_1 = Delta_q_l[1];

                            DeltaJoint_r_0 = Delta_q_r[0];
                            DeltaJoint_r_1 = Delta_q_r[1];

                            // left
                            commands_left[0]  =  encoders_left[0] + alpha * DeltaJoint_l_0;
                            commands_left[1]  =  encoders_left[1] + alpha * DeltaJoint_l_1; //0.00;
                            commands_left[2]  =  encoders_left[2] + 0.00;
                            commands_left[3]  =  encoders_left[3] - 2.0 * alpha * DeltaJoint_l_0;
                            commands_left[4]  =  encoders_left[4] - alpha * DeltaJoint_l_0;
                            commands_left[5]  =  encoders_left[5] - alpha * DeltaJoint_l_1; //0.00;
                            // right
                            commands_right[0] =  encoders_right[0] + alpha * DeltaJoint_r_0;
                            commands_right[1] =  encoders_right[1] + alpha * DeltaJoint_r_1; //0.00;
                            commands_right[2] =  encoders_right[2] + 0.00;
                            commands_right[3] =  encoders_right[3] - 2.0 * alpha * DeltaJoint_r_0;
                            commands_right[4] =  encoders_right[4] - alpha * DeltaJoint_r_0;
                            commands_right[5] =  encoders_right[5] - alpha * DeltaJoint_r_1; //0.00;

                        break; 
                    }

                    case 1: 
                    {
                            DeltaJoint_l_0 = -0.5* Delta_q_l[3];
                            DeltaJoint_l_1 = 0.* Delta_q_l[1];

                            DeltaJoint_r_0 = -0.5* Delta_q_r[3];
                            DeltaJoint_r_1 = 0.* Delta_q_r[1];

                            // left
                            commands_left[0]  =  encoders_left[0] + alpha * Delta_q_l[0];
                            commands_left[1]  =  encoders_left[1] + alpha * Delta_q_l[1]; //0.00;
                            commands_left[2]  =  encoders_left[2] + alpha * Delta_q_l[2];
                            commands_left[3]  =  encoders_left[3] + alpha * Delta_q_l[3];
                            commands_left[4]  =  encoders_left[4] + alpha * Delta_q_l[4];
                            commands_left[5]  =  encoders_left[5] + alpha * Delta_q_l[5]; //0.00;
                            // right
                            commands_right[0] =  encoders_right[0] + alpha * Delta_q_r[0];
                            commands_right[1] =  encoders_right[1] + alpha * Delta_q_r[1]; //0.00;
                            commands_right[2] =  encoders_right[2] + alpha * Delta_q_r[2];
                            commands_right[3] =  encoders_right[3] + alpha * Delta_q_r[3];
                            commands_right[4] =  encoders_right[4] + alpha * Delta_q_r[4];
                            commands_right[5] =  encoders_right[5] + alpha * Delta_q_r[5]; //0.00;

                        break;  
                    }

                };  

                // ============ write the commands ====================
                        //           
                        ipos_left->positionMove(4, commands_left[4]);
                        ipos_right->positionMove(4, commands_right[4]);

                        Time::delay(0.002);

                        ipos_left->positionMove(3, commands_left[3]);
                        ipos_right->positionMove(3, commands_right[3]);

                        Time::delay(0.002);

                        ipos_left->positionMove(commands_left.data());
                        ipos_right->positionMove(commands_right.data());

                        bool done_l=false;
                        bool done_r=false;
                             while(!done_l && !done_r)
                             {
                                 ipos_left->checkMotionDone(&done_l);
                                 ipos_right->checkMotionDone(&done_r);
                                 Time::delay(0.02);
                             }
                
        } // for nb_iter

    }

}


void RobotPostures::moveToinitialWalkingPosture(        IEncoders *iencs_left,
                                                        IEncoders *iencs_right,
                                                 IPositionControl *ipos_left,
                                                 IPositionControl *ipos_right,
                                                yarp::sig::Vector JointsOffset,
                                                yarp::sig::Vector Des_ljts,
                                                yarp::sig::Vector Des_rjts)
{
    // 
    int ljoints, rjoints;
    iencs_left->getAxes(&ljoints);
    iencs_right->getAxes(&rjoints);

    yarp::sig::Vector Des_ljoints, Des_rjoints;
    Des_ljoints.resize(ljoints);
    Des_rjoints.resize(rjoints);

    double Desqpitch = 27.00; // 27.0  //26
    //
    Des_ljoints[0] =  Desqpitch+0.00; // off_set: 0.00
    Des_ljoints[1] =  -0.00;  // 1.00
    Des_ljoints[2] =  0.00;
    Des_ljoints[3] = -2.*Desqpitch;
    Des_ljoints[4] = -Desqpitch;
    Des_ljoints[5] =  0.00;   // -1.00

    //
    Des_rjoints[0] =  Desqpitch+0.00;
    Des_rjoints[1] =  -0.00;
    Des_rjoints[2] =  0.00;
    Des_rjoints[3] = -2.*Desqpitch;
    Des_rjoints[4] = -Desqpitch;  //27
    Des_rjoints[5] =  0.00;
    //

    RobotPostures::moveToLegsPosture( iencs_left,
                                      iencs_right,
                                      ipos_left,
                                      ipos_right,
                                      JointsOffset,
                                      Des_ljoints,
                                      Des_rjoints);

}

void RobotPostures::moveTofinalWalkingPosture(  IEncoders *iencs_left,
                                                IEncoders *iencs_right,
                                         IPositionControl *ipos_left,
                                         IPositionControl *ipos_right,
                                                   Vector JointsOffset,
                                        yarp::sig::Vector Des_ljoints,
                                        yarp::sig::Vector Des_rjoints)
{
    // 
    int ljoints, rjoints;
    iencs_left->getAxes(&ljoints);
    iencs_right->getAxes(&rjoints);

    yarp::sig::Vector Desq_l, Desq_r;
    Desq_l.resize(ljoints);
    Desq_r.resize(rjoints);

    double Desqpitch = 27.00; // 27.0  //26
    //
    Desq_l[0] =  0.00 + 2.50; //-1.50
    Desq_l[1] = -0.70;
    Desq_l[2] =  0.00;
    Desq_l[3] = -4.00;
    Desq_l[4] =  0.00 - 2.50;  //-1.50
    Desq_l[5] =  0.50;

    //
    Desq_r[0] =  0.00 + 2.50; //-1.50
    Desq_r[1] = -0.70;
    Desq_r[2] =  0.00;
    Desq_r[3] = -4.00;
    Desq_r[4] =  0.00 - 2.50; //-1.50
    Desq_r[5] =  0.70;
    //

    RobotPostures::moveToLegsPosture( iencs_left,
                                      iencs_right,
                                      ipos_left,
                                      ipos_right,
                                      JointsOffset,
                                      Desq_l,
                                      Desq_r);
}

void RobotPostures::moveToHeightBasedPosture(       IEncoders *iencs_left,
                                                    IEncoders *iencs_right,
                                             IPositionControl *ipos_left,
                                             IPositionControl *ipos_right,
                                                       Vector JointsOffset,
                                                       double CoMHeight)
{
    // Create iCubLeg objects for the two legs
    iCub::iKin::iCubLeg limb_L_Leg("left_v2.5");   // left leg  
    iCub::iKin::iCubLeg limb_R_Leg("right_v2.5");  // right leg

    // Create kinematic chains for the legs
    iCub::iKin::iKinChain *LeftLegChain;
    iCub::iKin::iKinChain *RightLegChain;   // MAY Be Should be declared as members of the class (TO ANALYSE)

    LeftLegChain  = limb_L_Leg.asChain();
    RightLegChain = limb_R_Leg.asChain();

    // Vectors of desired legs joints
    yarp::sig::Vector DesiredLeftLeg_joints;
    yarp::sig::Vector DesiredRightLeg_joints;

    DesiredLeftLeg_joints.resize(LeftLegChain->getDOF(), 0.0);
    DesiredRightLeg_joints.resize(RightLegChain->getDOF(), 0.0);


    yarp::sig::Vector DesiredLeftLegPoseAsAxisAngles;
    yarp::sig::Vector DesiredRightLegPoseAsAxisAngles;

    DesiredLeftLegPoseAsAxisAngles.resize(7, 0.0);
    DesiredRightLegPoseAsAxisAngles.resize(7, 0.0);

    // Setting values for the initial desired pose
    DesiredLeftLegPoseAsAxisAngles(0) =  0.00;
    DesiredLeftLegPoseAsAxisAngles(1) = -0.068;
    DesiredLeftLegPoseAsAxisAngles(2) = -(0.10 + CoMHeight);
    DesiredLeftLegPoseAsAxisAngles(3) =  0.00;
    DesiredLeftLegPoseAsAxisAngles(4) = -1.00;
    DesiredLeftLegPoseAsAxisAngles(5) =  0.00;
    DesiredLeftLegPoseAsAxisAngles(6) =  M_PI/2;

    DesiredRightLegPoseAsAxisAngles(0) =  0.00;
    DesiredRightLegPoseAsAxisAngles(1) =  0.068;
    DesiredRightLegPoseAsAxisAngles(2) = -(0.10 + CoMHeight);
    DesiredRightLegPoseAsAxisAngles(3) =  0.00;
    DesiredRightLegPoseAsAxisAngles(4) = -1.00;
    DesiredRightLegPoseAsAxisAngles(5) =  0.00;
    DesiredRightLegPoseAsAxisAngles(6) =  M_PI/2;

    // instantiate a IPOPT solver for inverse kinematic
    // for both translational and rotational part
    iCub::iKin::iKinIpOptMin *left_leg_solver;
    iCub::iKin::iKinIpOptMin *right_leg_solver;

    // inverse Kinematics
    // left
    left_leg_solver  = new iCub::iKin::iKinIpOptMin( *LeftLegChain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    left_leg_solver->setUserScaling(true,100.0,100.0,100.0);
    // right
    right_leg_solver = new iCub::iKin::iKinIpOptMin(*RightLegChain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    right_leg_solver->setUserScaling(true,100.0,100.0,100.0);
    //
    int ljoints, rjoints;
    iencs_left->getAxes(&ljoints);
    iencs_right->getAxes(&rjoints);

    // create vetors to contain the encoders and commands values
    yarp::sig::Vector  encoders_left, commands_left;
    yarp::sig::Vector  encoders_right, commands_right;
        // left
    encoders_left.resize(ljoints);
    commands_left.resize(ljoints);
        // right
    encoders_right.resize(rjoints);
    commands_right.resize(rjoints);

    // left leg
    iencs_left->getEncoders(encoders_left.data());
    LeftLegChain->setAng(CTRL_DEG2RAD*encoders_left);

    // right leg
    iencs_right->getEncoders(encoders_right.data());
    RightLegChain->setAng(CTRL_DEG2RAD*encoders_right);

    Time::delay(0.020);

    // left leg
    ipos_left->positionMove((encoders_left).data());
    // right
    ipos_right->positionMove((encoders_right).data());

    double DeltaJoint_l_0, DeltaJoint_r_0;
    double DeltaJoint_l_1, DeltaJoint_r_1;

     Vector Delta_q_l, Delta_q_r;
		       Delta_q_l.resize(ljoints);
		       Delta_q_r.resize(rjoints);

    // 
    int nb_iter = 30;
    double alpha;
           alpha = 0;

    for (int ii=0; ii<2; ii++)
    {
         // getting actual joints values
        while(!iencs_left->getEncoders(encoders_left.data()))
        {
              Time::delay(0.01);
              printf(".");
        };
        while(!iencs_right->getEncoders(encoders_right.data()))
        {
              Time::delay(0.01);
              printf(".");
        };

        // update the Kinematics model
        LeftLegChain->setAng(CTRL_DEG2RAD*encoders_left);
        RightLegChain->setAng(CTRL_DEG2RAD*encoders_right);

        // Compute the desired joints
        // left leg
        DesiredLeftLeg_joints  = left_leg_solver->solve(LeftLegChain->getAng(), DesiredLeftLegPoseAsAxisAngles);
        // right leg
        DesiredRightLeg_joints = right_leg_solver->solve(RightLegChain->getAng(), DesiredRightLegPoseAsAxisAngles);

        // compute the joints position errors
            for (int k=0; k<ljoints; k++)
            {
                Delta_q_l[k] = CTRL_RAD2DEG*DesiredLeftLeg_joints[k] - encoders_left[k];
                Delta_q_r[k] = CTRL_RAD2DEG*DesiredRightLeg_joints[k] - encoders_right[k];
            }

        
        for(int i=0; i<nb_iter+1; i++)
        {
            alpha = 3.*pow(i,2.)/pow(nb_iter, 2.)-2.*pow(i,3.)/pow(nb_iter, 3.);

            switch (ii)
                {
                    case 0: 
                    {
                            DeltaJoint_l_0 = Delta_q_l[0];
                            DeltaJoint_l_1 = Delta_q_l[1];

                            DeltaJoint_r_0 = Delta_q_r[0];
                            DeltaJoint_r_1 = Delta_q_r[1];

                            // left
                            commands_left[0]  =  encoders_left[0] + alpha * DeltaJoint_l_0;
                            commands_left[1]  =  encoders_left[1] + alpha * DeltaJoint_l_1; //0.00;
                            commands_left[2]  =  encoders_left[2] + 0.00;
                            commands_left[3]  =  encoders_left[3] - 2.0 * alpha * DeltaJoint_l_0;
                            commands_left[4]  =  encoders_left[4] - alpha * DeltaJoint_l_0;
                            commands_left[5]  =  encoders_left[5] - alpha * DeltaJoint_l_1; //0.00;
                            // right
                            commands_right[0] =  encoders_right[0] + alpha * DeltaJoint_r_0;
                            commands_right[1] =  encoders_right[1] + alpha * DeltaJoint_r_1; //0.00;
                            commands_right[2] =  encoders_right[2] + 0.00;
                            commands_right[3] =  encoders_right[3] - 2.0 * alpha * DeltaJoint_r_0;
                            commands_right[4] =  encoders_right[4] - alpha * DeltaJoint_r_0;
                            commands_right[5] =  encoders_right[5] - alpha * DeltaJoint_r_1; //0.00;

                        break; 
                    }

                    case 1: 
                    {
                            DeltaJoint_l_0 = -0.5* Delta_q_l[3];
                            DeltaJoint_l_1 =  0. * Delta_q_l[1];

                            DeltaJoint_r_0 = -0.5* Delta_q_r[3];
                            DeltaJoint_r_1 =  0. * Delta_q_r[1];

                            // left
                            commands_left[0]  =  encoders_left[0] + alpha * Delta_q_l[0];
                            commands_left[1]  =  encoders_left[1] + alpha * Delta_q_l[1]; //0.00;
                            commands_left[2]  =  encoders_left[2] + alpha * Delta_q_l[2];
                            commands_left[3]  =  encoders_left[3] + alpha * Delta_q_l[3];
                            commands_left[4]  =  encoders_left[4] + alpha * Delta_q_l[4];
                            commands_left[5]  =  encoders_left[5] + alpha * Delta_q_l[5]; //0.00;
                            // right
                            commands_right[0] =  encoders_right[0] + alpha * Delta_q_r[0];
                            commands_right[1] =  encoders_right[1] + alpha * Delta_q_r[1]; //0.00;
                            commands_right[2] =  encoders_right[2] + alpha * Delta_q_r[2];
                            commands_right[3] =  encoders_right[3] + alpha * Delta_q_r[3];
                            commands_right[4] =  encoders_right[4] + alpha * Delta_q_r[4];
                            commands_right[5] =  encoders_right[5] + alpha * Delta_q_r[5]; //0.00;

                        break;  
                    }

                };  

                // ============ write the commands ====================
                        //  
                        commands_left  += JointsOffset;
                        commands_right += JointsOffset;

                        ipos_left->positionMove(commands_left.data());
                        ipos_right->positionMove(commands_right.data());

                        bool done_l=false;
                        bool done_r=false;
                        while(!done_l && !done_r)
                        {
                             ipos_left->checkMotionDone(&done_l);
                             ipos_right->checkMotionDone(&done_r);
                             Time::delay(0.02);
                        }              
        } // for nb_iter

    }
    //
    Time::delay(0.1);
    //
    if (left_leg_solver){
        delete left_leg_solver;
        left_leg_solver = 0;

    }
    //
    if (right_leg_solver){
        delete right_leg_solver;
        right_leg_solver = 0;

    }

}



// // ===============================================================================================

// // ===============================================================================================

CommandsWriter::CommandsWriter() {}
CommandsWriter::~CommandsWriter(){}

void CommandsWriter::InitCommendWriter(InitBalWlkParameters       *Parameters,
                                       ControlledDevices          &botDevices)//,
                                       //CubicInterpolator    CmdSmoother)
{
    
    SendLegsCmds    = Parameters->ExecuteWalking;
    SendArmsCmds    = Parameters->ExecuteGrasping;
    useCmdOffset    = Parameters->useOffset;
    useCmdSmoother  = Parameters->SmoothCmd;
    nbInterpolPts   = Parameters->NbInterpolationPts;

    HipRollFactor   = Parameters->HipRollFactor;
    HipYawFactor    = Parameters->HipYawFactor;
    AnkleRollFactor = Parameters->AnkleRollFactor;

    // additional parameters for the manipulation (arms)
    useVelocityIK   = Parameters->VelocityIK;
    delta_theta_max = Parameters->Delta_theta_max;

    // Legs
    // commands_left_leg.resize(botDevices.lljoints);
    // commands_right_leg.resize(botDevices.rljoints); 

    previous_commands_left_leg.resize(botDevices.lljoints);
    previous_commands_right_leg.resize(botDevices.rljoints); 

    Sent_commands_left_leg.resize(botDevices.lljoints);
    Sent_commands_right_leg.resize(botDevices.rljoints);

    // Arms
        // position commands 
    // commands_left_arm.resize(botDevices.lajoints);
    // commands_right_arm.resize(botDevices.rajoints); 
    //     // torque commands
    // torqueCmd_left_arm.resize(botDevices.lajoints);
    // torqueCmd_right_arm.resize(botDevices.rajoints);

    previous_commands_left_arm.resize(botDevices.lajoints);
    previous_commands_right_arm.resize(botDevices.rajoints); 

    Sent_commands_left_arm.resize(botDevices.lajoints);
    Sent_commands_right_arm.resize(botDevices.rajoints);

    // torso
    // commands_torso.resize(botDevices.torsojoints);

    // initialization
    botDevices.iencs_left_leg->getEncoders(botDevices.commands_left_leg.data());
    botDevices.iencs_right_leg->getEncoders(botDevices.commands_right_leg.data());

    previous_commands_left_leg  = botDevices.commands_left_leg;
    previous_commands_right_leg = botDevices.commands_right_leg;

    Sent_commands_left_leg  = botDevices.commands_left_leg;
    Sent_commands_right_leg = botDevices.commands_right_leg;

    // initialization
    botDevices.iencs_left_arm->getEncoders(botDevices.commands_left_arm.data());
    botDevices.iencs_right_arm->getEncoders(botDevices.commands_right_arm.data());

    previous_commands_left_arm  = botDevices.commands_left_arm;
    previous_commands_right_arm = botDevices.commands_right_arm;

    Sent_commands_left_arm  = botDevices.commands_left_arm;
    Sent_commands_right_arm = botDevices.commands_right_arm;

    // Initialisation of the direct command smoother
    this->CmdSmoother.IncreasingInterpolation(nbInterpolPts);
    
}

void CommandsWriter::WriteLegsCommands(yarp::sig::Vector DesiredLeftLeg_joints, 
                                       yarp::sig::Vector DesiredRightLeg_joints, 
                                       ControlledDevices &botDevices)  
                                       //CubicInterpolator CmdSmoother,
                                       
{
    
    botDevices.commands_left_leg     = CTRL_RAD2DEG * DesiredLeftLeg_joints;
    botDevices.commands_left_leg[1] *= HipRollFactor;
    botDevices.commands_left_leg[2] *= HipYawFactor;
    botDevices.commands_left_leg[5] *= AnkleRollFactor* HipRollFactor;

    botDevices.commands_right_leg     = CTRL_RAD2DEG * DesiredRightLeg_joints;
    botDevices.commands_right_leg[1] *= HipRollFactor;
    botDevices.commands_right_leg[2] *= HipYawFactor;
    botDevices.commands_right_leg[5] *= AnkleRollFactor* HipRollFactor;


    if(SendLegsCmds)
    {
        if(!useCmdSmoother)
        {
            //
            for (int i=0; i< botDevices.lljoints; i++)
            {
                botDevices.ipos_left_legDir->setPosition(i, botDevices.commands_left_leg[i]);
                botDevices.ipos_right_legDir->setPosition(i,botDevices.commands_right_leg[i]);
            }
            Time::delay(0.005);
        }
        else
        {
            //
            for (int i=0; i<this->CmdSmoother.nb_of_points; i++)
            {
                Sent_commands_left_leg  = previous_commands_left_leg  + this->CmdSmoother.Up_Index(i) * (botDevices.commands_left_leg  - previous_commands_left_leg);
                Sent_commands_right_leg = previous_commands_right_leg + this->CmdSmoother.Up_Index(i) * (botDevices.commands_right_leg - previous_commands_right_leg);

                for (int j=0; j< botDevices.lljoints; j++)
                {
                    botDevices.ipos_left_legDir->setPosition(j, Sent_commands_left_leg[j]);
                    botDevices.ipos_right_legDir->setPosition(j,Sent_commands_right_leg[j]);
                }
                Time::delay(0.0005);
            }

        }
        // Update of the previoous commands
        previous_commands_left_leg  = botDevices.commands_left_leg;
        previous_commands_right_leg = botDevices.commands_right_leg;
    }

}

// Arms
void CommandsWriter::WriteArmsCommands( yarp::sig::Vector DesiredLeftArm_joints, 
                                        yarp::sig::Vector DesiredRightArm_joints,
                                        ControlledDevices &botDevices) 
                                        //CubicInterpolator CmdSmoother, 
                                        
{
    botDevices.commands_left_arm  = CTRL_RAD2DEG * DesiredLeftArm_joints;
    botDevices.commands_right_arm = CTRL_RAD2DEG * DesiredRightArm_joints;

    //
    yarp::sig::Vector Delta_l_arm_cmd(botDevices.lajoints);
    yarp::sig::Vector Delta_r_arm_cmd(botDevices.rajoints);


    if(SendArmsCmds)
    {
        if(!useCmdSmoother)
        {
            //
            for (int i=0; i< botDevices.lajoints; i++)
            {
                botDevices.ipos_left_armDir-> setPosition(i, botDevices.commands_left_arm[i]);
                botDevices.ipos_right_armDir->setPosition(i, botDevices.commands_right_arm[i]);
            }
            Time::delay(0.005);
        }
        else
        {
            //
            Delta_l_arm_cmd = botDevices.commands_left_arm  - previous_commands_left_arm;
            Delta_r_arm_cmd = botDevices.commands_right_arm - previous_commands_right_arm;

            // 
            if(!useVelocityIK)
            {
                for (int j=0; j<botDevices.lajoints; j++)
                {
                    // left
                    if(fabs(Delta_l_arm_cmd[j]) > delta_theta_max)
                    {
                        Delta_l_arm_cmd[j] = (1.0/fabs(Delta_l_arm_cmd[j]))*Delta_l_arm_cmd[j] * delta_theta_max;
                    }
                    // right
                    if(fabs(Delta_r_arm_cmd[j]) > delta_theta_max)
                    {
                        Delta_r_arm_cmd[j] = (1.0/fabs(Delta_r_arm_cmd[j]))*Delta_r_arm_cmd[j] * delta_theta_max;
                    }
                }
            }
            //
            for (int i=0; i<this->CmdSmoother.nb_of_points; i++)
            {
                Sent_commands_left_arm  = previous_commands_left_arm  + this->CmdSmoother.Up_Index(i) * (Delta_l_arm_cmd);
                Sent_commands_right_arm = previous_commands_right_arm + this->CmdSmoother.Up_Index(i) * (Delta_r_arm_cmd);

                for (int j=0; j< botDevices.lajoints; j++)
                {
                    botDevices.ipos_left_armDir->setPosition(j, Sent_commands_left_arm[j]);
                    botDevices.ipos_right_armDir->setPosition(j, Sent_commands_right_arm[j]);

                }

                Time::delay(0.0005);
            }
            
        }
        // Update of the previoous commands
        previous_commands_left_arm  += Delta_l_arm_cmd;
        previous_commands_right_arm += Delta_r_arm_cmd;

    }

}


void CommandsWriter::WriteArmsVelocityCommands( Eigen::VectorXd des_jts_velocity_left_arm, 
                                                Eigen::VectorXd des_jts_velocity_right_arm,
                                                ControlledDevices &botDevices) 
{
    //
    for (int j=0; j< 7; j++)
    {
        botDevices.VelocityCmd_left_arm[j]  = CTRL_RAD2DEG * des_jts_velocity_left_arm(j);
        botDevices.VelocityCmd_right_arm[j] = CTRL_RAD2DEG * des_jts_velocity_right_arm(j);
    }
    
    // //
    // for (int j=0; j< 7; j++)
    // {
    //     botDevices.ivel_left_arm->velocityMove(j, botDevices.VelocityCmd_left_arm[j]);
    //     botDevices.ivel_right_arm->velocityMove(j, botDevices.VelocityCmd_right_arm[j]);
    // }
 
        botDevices.ivel_left_arm->velocityMove(botDevices.VelocityCmd_left_arm.data());
        botDevices.ivel_right_arm->velocityMove(botDevices.VelocityCmd_right_arm.data());
    
    Time::delay(0.0005);

}