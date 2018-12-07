# biped-walking-controller

This module implements a reactive omnidirectional walking controller for the biped humanoid robot iCub.

Given desired velocity (v<sub>x</sub>, v<sub>y</sub>, w<sub>z</sub>) of the center of mass (CoM) of the robot (actually the frame attached the the CoM), the controller compute and execute the footstep positions and orientation required to realized the desired velocity of the CoM.

The robot under this walking controller can be driven in two mode: 
- Direct velocity based reactive walking mode  or 
- Interaction forces based reactive walking (where an admittance is used to convert the applied forces into velocity)

For more details see [Capture-point based balance and reactive omnidirectional walking controller](https://ieeexplore.ieee.org/document/8239532)

---

### System Requirements
Ubuntu 16.04 and Gazebo >=7

### Dependencies
Prior to the compilation of this controller, make sure you have installed the following software:
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main) (Install from sources!)
- [Eigen3](): Eigen 3 version >=3.2.9 is necessary for [yarpWholeBodyInterface](https://github.com/robotology/yarp-wholebodyinterface), you should install it from source, follow instructions [here](https://github.com/eigenteam/eigen-git-mirror)
- [qpOASES](https://projects.coin-or.org/qpOASES/wiki/QpoasesInstallation)
- [yarpWholeBodyInterface](https://github.com/robotology/yarp-wholebodyinterface)

For simulation
- [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install): Gazebo7 and libgazebo7: [installation instructions](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) 
- [gazebo_yarp_plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [icub-gazebo](https://github.com/robotology/icub-gazebo)
- [GetLinkWorldPose](https://github.com/epfl-lasa/GetLinkWorldPose.git)

#### Easier Installation option (on clean Ubuntu 16.01 - recommended):
1. Install latest version of Eigen3 as above.
2. Install Gazebo7 and libgazebo7: [installation instructions](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) 
3. Install all yarp/iCub/gazebo-plugin libraries with [robotology-superbuild](https://github.com/robotology/robotology-superbuild)
4. Install and compile the [GetLinkWorldPose](https://github.com/epfl-lasa/GetLinkWorldPose.git) Gazebo plugin, in order to simulate DS-based motion planning.

---

### Compilation and build

Clone the repository
```bash
$ git clone https://github.com/epfl-lasa/biped-walking-controller.git
```
Edit first the `CMakeLists.txt` file to indicate :
- the correct paths of qpOASES lib file (`libqpOASES.so`) and qpOASES directory
- the correct path to eigen3

Once the CMakeList.txt edited, the controller can be built. Just run

```bash
$ cd ~/biped-walking-controller
$ mkdir build && cd build
$ cmake .. && make
```
This will create three executables which will placed in ``~/biped-walking-controller/build``.
- ``./BipedWalkingGrasping``: executable that runs the walking controller with different desired CoM velocity commands
- ``./BipedWalkingGrasping_ROS``: executable that runs the walking controller with DS LfD via ROS connections
- ``./KeyBoardCommandReader``: executable that runs a keyboard-command-reader in which the user can set the desired velocity with keyboard commands.

---

### Running the controller

- **Terminal 1** Start yarpserver:
```bash
$ yarpserver
```
- **Terminal 2** (simulation) start gazebo simulator and import include the robot model (`iCub (no hands)`)
```bash
$ gazebo 
```
- **(Optional)** In another terminal, bring the robot in home position 
```bash
$ yarpmotorgui --from homePoseBalancing.ini --robot robot_name 
```
robot_name: (e.g. icub or icubSim) and then press the 'Home All' button

- **Terminal 3** Launch the keyboard command reader as follows : 
```bash
$ ./KeyboardCommandsReader --from ../config/BalanceWalkingController.ini
```
- **Terminal 4** In another terminal launch the walking controller as follows : 
```bash
$ ./BipedWalkingGrasping --from ../config/BalanceWalkingController.ini
```

#### Testing different walking commands
We currently have 3 different ways of generating desired CoM velocity (v<sub>x</sub>, v<sub>y</sub>, w<sub>z</sub>). These types and their parameters can be defined in the config file: ``BalanceWalkingController.ini`` like so,
```
# Control Type 0: Fixed initial velocity, 1: Using Keyboard increments, 2: Using a linear DS 
VelocityCmdType		2
```
 1. Fixed Velocity: ``VelocityCmdType		0``
 ```
   # Initial velocity Vx [m/s], Vy [m/s],  Wz [rad/s], 
   VelocityX    0.01
   VelocityY    0.00
   OmegaZ       0.00
 ```

 2. Command Velocity via Keyboard increments: ``VelocityCmdType		1``
 ```
   variation 0.005
 ```
 - Arrow keys: Define linear velocity increments
 - 'a' and 'd' keys: Define angular velocity increment
 
 3. Desired Velocity will be generated via a simple linear DS: ``VelocityCmdType		2``   
 The implemented DS is of the form <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/linear_DS.gif"> whose parameters can be defined as follows:
  ```
    # Desired Target with linear DS x [m], y [m], z [m] 
    kappa      0.2
    AttractorX 2.00
    AttractorY -1.00
    AttractorZ 0.541591
  ```  
   where:
    - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/CoM.gif">: CoM position
    - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/attractor.gif">: Attractor (target)
    - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/kappa.gif">: DS gain     

   The angular velocity <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega_z.gif"> is defined with the following equation: <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega_eq.gif">, where:  
    
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/R.gif">:  Current Rotation matrix of the robot's CoM in world reference frame
    - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/R_d.gif">: Desired Rotation matrix of the robot's CoM in world reference frame, computed by aligning R with the direction of motion given by the DS <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/ds_dir.gif">
    - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega_skew.gif">: The skew-symmetric matrix representing the angular velocity vector <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega.gif">
   
   For this test, the ```./KeyboardCommandsReader``` is not necessary.  
   
   ***NOTE: To test the ROS interface that generates CoM velocity based on non-linear DS LfD, follow the instructions in [icub-ds-walking](https://github.com/epfl-lasa/icub-ds-walking.git)***


#### Known Run-time Issues (and solutions)
- If you get the error `did not find model.urdf` you must replace, `model.urdf` in the `BalanceWalkingController.ini` config file with `~/robotology-superbuild/build/install/share/codyco/robots/icubGazeboSim/model.urdf`
---


## Expected behavior

The behavior of the controller is  demonstrated [here](https://www.youtube.com/watch?v=9hKOVHDDnfc&t=16s)

---

## Controller details


### Configuration file

The configuration file is `BalanceWalkingController.ini` locate in the `config` folder. The parameters are

- `robot`: name of the robot to connect to (e.g icubSim for simulation and icub for the real robot)
- `name`: name of the module. All ports open by the controller will include this name (default: walkingGrasping)
- `wbi_config_file`: name of the configuration file of yarpWholeBodyInterface (yarpWholeBodyInterface.ini)
- `wbi_joint_list`: list of joints within the `wbi_config_file` to be loaded by the wholeBodyInterface (default: ROBOT_DYNAMIC_MODEL_JOINTS)
- `period`: period of the controller thread (default 40 ms)
- `modulePeriod`: period of the module
- `duration`: running time duration in secondes, once reached the robot stops and goes back to its initial standing posture

- `FT_feedback`: mode of reactive walking (0: direct velocity mode with no force interaction; 1: admittance control using feet forces/moments and 2: admittance using measured arms forces/momemts)

- `VelocityX`: the initial forward or backward velocity of the robot in [m/s]
- `VelocityY`:  the initial lateral velocity of the robot in [m/s]
- `OmegaZ`:	the initial rotational velocity of the robot about the vertical axis in [m/s]	


### Other parameters

Others parameters are in the `src/InitBalWlkParameters.cpp` file. 

#### Citing this contribution
In case you want to cite the content of this controller implementation, please refer to [Capture-point based balance and reactive omnidirectional walking controller](https://ieeexplore.ieee.org/document/8239532) and use the following bibtex entry:

``` 
 @inproceedings{bombile2017capture,
  title={Capture-point based balance and reactive omnidirectional walking controller},
  author={Bombile, Michael and Billard, Aude},
  booktitle={Humanoid Robotics (Humanoids), 2017 IEEE-RAS 17th International Conference on},
  pages={17--24},
  year={2017},
  organization={IEEE}
}
```

---

#### Known Installation Issues (and solutions) for Ubuntu 14.04 [Not working, Ubuntu 16 is needed!]
- YARP needs CMake version >3.5, if you have Ubuntu 14.04, this can be installed by following the instructions [here](https://www.claudiokuenzler.com/blog/755/install-upgrade-cmake-3.10.1-ubuntu-14.04-trusty-alternatives#.XAKUHxgnapo)
- If you're trying to install YARP in Ubuntu 14.04 with ROS-Indigo installed in it, you will probably get a compiling error regarding yarpcar_xmlrpc component, solution is found [here](https://github.com/robotology/yarp/issues/1323)
- GCC/G++ version >=5 is necessary, to install follow instructions [here](https://gist.github.com/beci/2a2091f282042ed20cda)
- [gazebo_yarp_plugins](https://github.com/robotology/gazebo-yarp-plugins) needs Boost version >=1.55, Ubuntu 14.04 comes with version 1.54. To upgrade boost by follow instructions [here](https://codeyarns.com/2013/12/27/how-to-upgrade-the-boost-library-on-ubuntu/).
***Update***: ``libgazebo7-dev`` which is needed to add plugins to gazebo only works with ``libboost-all-dev`` which becomes conflicted when installing ``libboost1.55-dev`` as it is hard-coded to point to ``libboost1.54-dev``. This was a waste of time. ***You NEED Ubuntu 16!*** Otherwise, if someone finds the solution, please write it here. -Nadia
