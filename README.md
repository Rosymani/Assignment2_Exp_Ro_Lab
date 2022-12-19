# Assignment2_Exp_Ro_Lab
# Experimental Robotics Laboratory Second Assignment
The second assignment builds on top of the first one: we students are now asked to implement a similar robot behaviour on a gazebo-simulated pet robot. The robot is akin to a dog, it's a wheeled robot with a neck and a camera-equipped head that can rotate: it can move randomly, track a ball, move its head and go to sleep.

## System Architecture

### Component Diagram

The software architecture is based on **four main components**:

- **Person module**

    This module mimics the behaviour of a person that can either move the ball to a random valid location or make it disappear under the plane. It implements an action client which randomly sends one of the two types of goals to the *"go_to_point_ball.py"* action server: the module then waits for the ball to reach its destination and sleeps for some time before sending another goal.

- **Finite state machine**

    This component implements a finite state machine using "Smach". It features an action client, two publishers and one subscriber. The three states, together with the transitions between them, will be further explained in the following paragraph.

- **Robot action server**
    This component implements an action server to control the robot. Since it was provided by the professor and I only modified some gains and some print functions, I'm not going to further explain how it works in detail.

- **Ball action server**

    This component implements an action server to control the ball. As for the robot action server, I'm not going over it in detail. 

### State Diagram



This is the state diagram that shows how the finite state machine works:

When the robot is in **Normal** state it moves randomly in the plane by sending a goal to the robot action server: if at anytime the robot sees the ball, then it switches to the *Play* state. If instead the robot has performed enough actions (an action is either reaching a location or performing the sequence of camera movements as described in the *Play* state) then it switches to the *Sleep* state.

When the robot is in **Sleep** state, it first reaches the predefined "Home" location (-5, 7), then stays there for some time and finally wakes up, transitioning back to the *Normal* state.

When the robot is in **Play** state, it tracks the ball until it gets close enough: when this happens, the robot stays still and rotates its head 45 degrees on the left, then does the same for the right and finally goes back to the center. This behaviour goes on until the ball can't be found for a certain period of time, after which the robot switches back to the *Normal* state.

## ROS Messages and Parameters

Custom ROS **messages** are:

 **Planning.action**

    ```
    geometry_msgs/PoseStamped target_pose
    ---
    ---
    string stat
    geometry_msgs/Pose position 
    ```

    Describes the action that will be used both for the robot action server and the ball one: `target_pose` represents the "goal", `stat` and `position` are the "feedback": there's no "result" field.

I haven't defined any custom **ROS parameter**.

---

## Packages and File List

Going in alphabetical order:

- **action**

    - `Planning.action`
    
        Action file described above.

- **config**

    - `motors_config.yaml`

        The file containing the description of the controllers and their parameters.

- **diagrams**

    - `Component_Diagram.png`, `State_Diagram.png` and `rosgraph.png`

        The three diagrams shown in this README file.

- **documentation**

    - **html** and **latex**

        Contain the output of *Doxygen*.

    - `assignment.conf`

        Configuration file used to run *Doxygen*.

- **launch**

    - `gazebo_world.launch`

        The launchfile which runs the gazebo server and client, the action servers, the finite state machine and spawns the ball, robot and human models.

- **scripts**

    - `go_to_point_ball.py`, `go_to_point_robot.py`, `person.py` and `state_machine.py`

        The modules of the architecture, as described in the *System Architecture* section.

- **urdf**

    - `ball.gazebo`, `ball.xacro`, `human.urdf`, `robot.gazebo` and `robot.xacro`

        The files which describe the models used for the ball, robot and human.

- **worlds**

    - `world_assignment.world`

    The world file in which the simulation environment is defined.

- `CMakeLists.txt` and `package.xml`

    Necessary files to compile and run the system nodes.


## Installation and Running Procedure

First of all, clone this repository inside your ROS workspace's *"/src"* folder .

Then, navigate to the *"/scripts"* folder and make the Python scripts executable with:
```
$ chmod +x go_to_point_ball.py
$ chmod +x go_to_point_robot.py
$ chmod +x person.py
$ chmod +x state_machine.py
```

Go back to the root folder of your ROS workspace and execute:
```
$ catkin_make
$ catkin_make install
```

In a separate terminal run:
```
$ roscore
```

Finally, run the launchfile with this command:
```
$ roslaunch erl_second_assignment gazebo_world.launch
```
After a brief setup period in which gazebo is launched and the models are spawned, on the console you will see the transitions between states, when the robot and the ball reach their destinations, and other feedbacks from the system.

---

## System's Features

This architecture is an evolution of the first assignment, with some improvements. 
The person can send commands to the ball ignoring completely the state of the robot, which will work in a reactive way.
The requirements have all been satisfied, the person module controls the ball without problems and the finite state machine switches correctly between states. The robot uses the camera information to track the ball and when it stops it performs the requested head movements.
Since the robot is a system on its own, if the person module or the ball action server fail or stop working no problems arise since it can still move randomly in the plane. This division makes the overall system modular and robust.
An important point is that now the control of the robot motion is achieved via an action server, which is a more flexible, non-blocking option with respect to a service/client pattern and features the possibility both to cancel the current goal and to receive a constant feedback. 
I observed no strange behaviours and the feedback on the console is consistent with what is happening in gazebo.

## Authors and Contacts
Polaka Surendra 
Matricula No 4846909
