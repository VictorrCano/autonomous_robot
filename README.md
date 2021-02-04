# ECSE211 Project: Team 04 - The Canadian Geese

The Fall 2020 ECSE 211 project has tasked us to design and program a Java-based virtual robot capable of autonomously navigating a given play-area and collect points via the pushing of blocks, or containers, up and ramp and into a bin. This robot will compete against another robot for points. The weight of the blocks pushed into the bin will determine the score of a robot. To that end, a robot will require the implementation of proper localization, navigation, and collision detection capabilities. 
## Team 
The project is undertaken by the individuals listed below. Each team member has a designated role, but float between different roles as well depending on priority levels. 

| Role    | Name |
| ---      | ---       |
| *Project Management* | Taylor Lynn Curtis |
| *Software Lead* | Benjamin Emiliani    |
| *Testing Lead* |Justin Legrand |
| *Hardware Lead* |Fahim Rezai |
| *Documentation Lead* | Edward Latulipe-Kang |
| *Technical Support* |Victor Cano |

## Requirements
The robot must be able to navigate a virtual world of 15' x 9' divided into a grid layout. The field is separated in three zones: red zone, green zone, and search zone. Each zone is separated by a river and connected by bridges. The search zone is shared physically by the two competing robots and any collision will result in
automatic disqualification for the robot that incurred the collision. The tentative minimum time limit is 5 minutes starting from the transmission of the parameters to completion of the task. The task is considered successful if the robot successfully pushes at least one container into the corresponding bin without colliding into the other robot

![Map](https://user-images.githubusercontent.com/22549112/99007737-cd5d1280-2512-11eb-9bdf-98b6fbe4f6a5.png)

## Hardware specifications
The robot is designed with solely lego components and an EV3 Lego Mindstorm kit. Presented below are the physical specifications of the components. It is important to realize that these are the constraints that the simulated digital twin attempts to adhere to. This is not perfect, and testing was done to ensure the validity of the relevant components. 
|Component| Description|
|---|---|
|*1 X EV3 brick* |Runs on a TI Sitara AM1808 (ARM9) @ 300 MHz|
|*2 X EV3 Large Motor* |Capable of 160-170 rpm, 20 Ncm Running torque, and 40 Ncm Stall torque |
|*2 X EV3 Ultrasonic Sensor* |Operates at maximum range of 250cm with accuracy of +/- 1cm|
|*1 X EV3 Color Sensor* |Detects black/blue/green/yellow/red/white/brown at a sample rate of 1kHz/sec|

The current project version is using hardware design **V11**, as shown below:

![Screenshot from 2020-12-02 16-22-59](https://user-images.githubusercontent.com/22549112/101118086-6a611780-35b6-11eb-8d93-7a90a872aba7.png)

## Software Design

The following flowchart describes the operation of the system in various phases. Each phase refers to important sub-divisions of the overall task. Phase 1 describes the robot's ability to find a bridge. Phase 2 refers to its ability to cross a bridge. Phase 3 is concerned with the robot's ability to score points. For the purpose of the demo, only Phase 1 and Phase 2 will be implemented. 

![flowchart_Final](https://user-images.githubusercontent.com/22549112/101118161-8fee2100-35b6-11eb-9f13-292c5a4de373.png)

## Technologies used
The software system is constrained by and constructed with the available software tools that follow.
|Tool|Description|
|---|---|
| *LeJOS*| Java Virtual Machine which allows for the EV3 to be programmed via Java |
| *Webots*| Open-source application that provides a complete development environment to model, program, and simulate robotics|
| *LeoCad*| Open-source CAD program used to design virtual 3D models built from Lego bricks |
