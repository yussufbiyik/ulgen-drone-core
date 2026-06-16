# drone-core

This repository contains the core software components for controlling a drone in a distributed swarm. It covers mission logic, flight controllers, and various other modules.

This was the code that brought us 6th place in TEKNOFEST 2025 Swarm UAV competiton. 

> [!IMPORTANT]
> I should mention: I coded both drone-core and the [GCS](https://github.com/yussufbiyik/ulgen-ground-control) alone in ~2 months, including a 2.5 week period with one eye swollen shut. Code quality declines towards the end, but it was field-tested and brought us 6th place.

## Project Structure

The project has the following folder structure:

* **controllers**: Contains different controller modules for the drone.
  * `xbee_controller.py`: Manages communication via the XBee module.
  * `mavsdk_controller.py`: Enables communication with the drone via MAVSDK.
  * `drone_controller.py`: Hosts frequently used drone operations in accordance with the step control mechanism.
  * `offboard_controller.py`: Contains PID and APF operations and hosts operations related to Offboard mode.
  * `step_controller.py`: Control mechanism that manages step-based movements and inter-drone synchronization.

* **core**: Hosts the core components.
  * `drone.py`: The main module where the drone is defined; other modules can access the drone through this module, the drone's max speed, PID values, etc. are set here.
  * `mission.py`: Defines the base classes and logic for missions, all missions are derived versions of this class.

* **missions**: Hosts the missions.

* **utils**: Auxiliary modules used by different parts of the system.
  * `apf.py`: Collision avoidance algorithm using the Artificial Potential Field method.
  * `pid.py`: Algorithm for progressing to the target point using PID.
  * `socket_communication.py`: Virtually simulates communication links using sockets.
  * `formation_utilities.py`: Hosts common functions required for formation etc. operations (lat_lon -> Meters etc.).

* **Launch Scripts**
  * `service.py`: Service script that runs missions together with the interface, also allows the use of the interface in simulation by connecting XBees to a single computer.
  * `tester.py`: Allows running missions for testing purposes with predefined parameters.

## Getting Started

For the simulation to work, PX4-Autopilot must be installed first.

### PX4 Installation
1. Clone the PX4 Repository
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

```

2. Run the Automatic Setup File

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

```

### Simulation with a Single Drone

To create a simulation environment with a Single Drone, use the `launch_drones.sh` script as in the example below

```bash
# For example:
# ./launch_drones.sh <number_of_drones>
./launch_drones.sh 1

```

### Simulation with Multiple Drones

Just like in the single drone simulation, use the `launch_drones.sh` script and enter the number of drones as you wish

```bash
# For example:
# ./launch_drones.sh <number_of_drones>
./launch_drones.sh 3

```

Gazebo will appear on your screen, to view the connection ports and codes, look at the terminal window where you ran the script, you will see the sample MAVSDK connection code for each drone.

I recommend creating a `sim_instance` like in the mission codes of the flight proof video to speed up drone selection.

PX4 Log records are located in the folder named `~/drone_logs`

**Supported Gazebo Versions:**

* Classic
* Harmonic

### Doing Simulation with the Interface

```bash
python service.py --drone_id <drone_id_value_in_simulation> --is_sim <is_simulation>

# For example, for the 1st drone in the simulation
python service.py --drone_id 0 --is_sim 1
# For the 2nd Drone
python service.py --drone_id 1 --is_sim 1
```

* `drone_id`: the variable progresses starting from 0 up to one less than the number of drones you added.
* `is_sim`: takes the value of 1 or 0, 1 corresponds to yes; 2 corresponds to no, asks whether the working environment is a simulation or not, sets the port used when connecting to the drone based on the given value.

A new terminal is opened for each drone and that drone is connected to by entering the drone's id value and working environment, and when the drones are listed on the interface, the mission parameters are set from the interface and the mission start command is given.

### Running Tests

To run any controller or mission module, you can use the `tester.py` script from the root directory of the project.

The script takes an argument in the form of the Python path of the module to be run.

**Usage format:**

```bash
python tester.py module.location drone_to_run(sim_instance)
```

**For example:**

To run the mission in the `./missions/ucus_kanit.py` path with the 1st drone:

```bash
python tester.py missions.formasyon 0
```

For the 2nd and 3rd drones, you can increase the number 0 as 1, 2...

## Known Bugs and Issues

* [x] Altitude drop during transition to formation (90% Fixed)
* [x] Following dangerous routes that are very close to other drones during transition to formation (90% Fixed)
* [x] Occasional PID lock-up at very close distances
* [x] Occasional lock-up problem experienced in altitude control even though its frequency has been reduced (May be due to the drone not being able to climb to the correct altitude or the altitude calculation being incorrect)
* [x] ID calculations of neighboring drones being problematic in XBee mode
* [x] The landing drone not being able to take off again in the add/remove individual mission
* [x] Both 2 drones sometimes receiving the order to leave the formation in the add/remove individual mission

## Completed Missions

* 3D Formation Mission
* [x] Simulation Tests
* [x] Field Tests


* Navigation with Formation Mission
* [x] Simulation Tests
* [x] Field Tests


* Add Remove Individual Mission
* [x] Simulation Tests
* [x] Field Tests (100%)
* [x] Field Tests
