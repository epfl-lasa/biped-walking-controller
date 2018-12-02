# biped-walking-controller

This module implements a reactive omnidirectional walking controller for the biped humanoid robot iCub.

Given desired velocity (v<sub>x</sub>, v<sub>y</sub>, w<sub>z</sub>) of the center of mass (CoM) of the robot (actually the frame attached the the CoM), the controller compute and execute the footstep positions and orientation required to realized the desired velocity of the CoM.

The robot under this walking controller can be driven in two mode: 
- Direct velocity based reactive walking mode  or 
- Interaction forces based reactive walking (where an admittance is used to convert the applied forces into velocity)

For more details see [Capture-point based balance and reactive omnidirectional walking controller](https://ieeexplore.ieee.org/document/8239532)

---
## System Requirements
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

**Easiest option (on clean installation):**
1. Install latest version of Eigen3 as above.
2. Install Gazebo7 and libgazebo7: [installation instructions](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) 
3. Install all yarp/iCub/gazebo-plugin libraries with [robotology-superbuild](https://github.com/robotology/robotology-superbuild)

#### Known Installation Issues (and solutions) for Ubuntu 14.04 [Not working, Ubuntu 16 is needed!]
- YARP needs CMake version >3.5, if you have Ubuntu 14.04, this can be installed by following the instructions [here](https://www.claudiokuenzler.com/blog/755/install-upgrade-cmake-3.10.1-ubuntu-14.04-trusty-alternatives#.XAKUHxgnapo)
- If you're trying to install YARP in Ubuntu 14.04 with ROS-Indigo installed in it, you will probably get a compiling error regarding yarpcar_xmlrpc component, solution is found [here](https://github.com/robotology/yarp/issues/1323)
- GCC/G++ version >=5 is necessary, to install follow instructions [here](https://gist.github.com/beci/2a2091f282042ed20cda)
- [gazebo_yarp_plugins](https://github.com/robotology/gazebo-yarp-plugins) needs Boost version >=1.55, Ubuntu 14.04 comes with version 1.54. To upgrade boost by follow instructions [here](https://codeyarns.com/2013/12/27/how-to-upgrade-the-boost-library-on-ubuntu/).
***Update***: ``libgazebo7-dev`` which is needed to add plugins to gazebo only works with ``libboost-all-dev`` which becomes conflicted when installing ``libboost1.55-dev`` as it is hard-coded to point to ``libboost1.54-dev``. This was a waste of time. ***You NEED Ubuntu 16!*** Otherwise, if someone finds the solution, please write it here. -Nadia

---

## Compilation and build

Clone the repository

```bash
$ git clone https://github.com/epfl-lasa/biped-walking-controller.git
```

### build
Edit first the `CMakeLists.txt` file to indicate :
- the correct paths of qpOASES lib file (`libqpOASES.so`) and qpOASES directory
- the correct path to eigen3

Once the CMakeList.txt edited, the controller can be built. Just run

```bash
$ cd ~/biped-walking-controller
$ mkdir build && cd build
$ cmake .. && make
```
This will create the ``./WalkingGrasping`` executable which runs the controller. It will be placed in ``~/biped-walking-controller/build``.

---

## Running the controller
Running this controller in its current version is still quite elaborate. 

- start yarpserver, in one terminal type the following
```
$ yarpserver
```
- (simulation) start gazebo simulator and import include the robot model (`iCub (no hands)`)
```
$ cd ~/robotology-superbuild/robotology
$ gazebo ./icub-gazebo/worlds/icub.world
```

- Bring the robot in home position 
```
$ yarpmotorgui --from homePoseBalancing.ini --robot robot_name 
```
robot_name: (e.g. icub or icubSim) and then press the 'Home All' button

- Launch the controller as follows : 
```
$ ./WalkingGrasping --from ../config/BalanceWalkingController.ini
```
This code will run the controller with a constant velocity (defined in the config file below) for a predetermined duration (in seconds, also defined in the config file).

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

```
Note: should you want to change the velocity at run time, you can do it in the while loop of the main.cpp file. 
An input port will be added for that purpose soon.
``` 


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
