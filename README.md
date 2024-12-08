
# Edabot Communication

Edabot Communication is a system designed to manage and simulate communication protocols between server and client components in a robotic framework. This repository is organized into two primary directories: `simulation` and `test`. The project also includes helper files for protocol development, communication handling, and utilities.

---

## Project Structure

### Directories

- **simulation/**  
  Contains resources and tools for simulating the robot's environment and ensuring communication integrity.  
  - **integrity/**: Handle all function for simulate integrity module.
  - `map.pgm`: A bitmap representation of the simulated environment.  
  - `map.yaml`: Configuration file for the map, linking the bitmap to the simulation.

- **test/**  
  Contains scripts and tools for testing the communication protocols between client and server.  
  - `__init__.py`: Marks the directory as a Python package.  
  - `client_to_server.py`: Tests basic communication from client to server.  
  - `server_to_client.py`: Tests basic communication from server to client.  
  - `ros_msg_dict_conversion.py`: Tests the conversion of ROS messages to Python dictionaries for easier manipulation.  
  - `orin_to_qinq`: Tests communication from orin to zinq

---

### Python Files

- **configs.py**  
  Defines configuration variables and settings for the project.  

- **edabot_protocol.py**  
  Implements the Edabot communication protocol, detailing message formats and communication standards.  

- **external_communication.py**  
  Manages communication between external systems and the robot.  

- **internal_communication.py**  
  Handles internal communication between different components of the robot's system.  

- **utils.py**  
  Contains utility functions to support other scripts, such as data conversion.

---


## How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/naufaldhaffaakbarwicaksono/edabot_communication.git
   ```
2. Set up the environment:
   ```bash
   cp .env.example .env
   ```
   Configure the `.env` file with your environment-specific settings.

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Run:
    - **For Jetson Orin**:

        ```bash
        python internal_communication.py
        ```
    - **For Raspi**:

        ```bash
        python external_communication.py
        ```

---


## Simulation

> **_NOTE:_**  Current simulation is only support for simulate communication between Jetson Orin and Zinq. To be updated soon!

**Setup Turtlebot3**
1. Install turtlebot3 simulation inside `edabot_communication` folder.

   ```bash
   cd simulation/src/
   git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
   cd .. && colcon build --symlink-install
   ```
2. Launch Simulation World

   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```
3. Open new terminal, and run Navigation Node

   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=simulation/map.yaml
   ```

> **_NOTE:_**  If you cannot open the map on rviz, you must change `robot_model_type` value to ` nav2_amcl::DifferentialMotionModel` inside `/opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml`.

**Run**
1. Simulate communication Jetson Orin and Zinq
   
   Open new terminal
   ```bash
   python -m simulation.integrity.simulate_zinq
   ```
   Open other new terminal
   ```bash
   python internal_communication.py
   ```

---


## Testing

- Test basic communication:
   ```bash
   python -m test.client_to_server
   ```
   ```bash
   python -m test.server_to_client
   ```

- Test ROS data conversion:
   ```bash
   python -m test.ros_msg_dict_conversion
   ```

---

