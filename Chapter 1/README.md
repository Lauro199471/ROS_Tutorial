# Chapter 1
In this chapter we will be talking about URDF Files

![robot1](https://user-images.githubusercontent.com/13907836/36332617-d0054cfa-1327-11e8-8f72-ed011af0fadc.PNG)
![r2d2](https://user-images.githubusercontent.com/13907836/36332618-d021f166-1327-11e8-84b8-2eb4b4b1a4c1.PNG)
*here are some simple examples of a URDF file*

![robonaut2-20-pound-2](https://user-images.githubusercontent.com/13907836/36332994-5c6ac9bc-1329-11e8-94ed-820b81de4f6a.jpg)
![rsz_seq_2-1024x429-1024x429](https://user-images.githubusercontent.com/13907836/36332927-18754318-1329-11e8-8f73-fb3eca02aeb8.png)
*NASA Robonaut in a URDF File and in real life*


**What are URDF files?** URDF files stand for *Unified Robot Description Format*. URDF is an XML format that describes a robot,its parts, its joints, dimensions, and so on. The URDF can represent the kinematic and dynamic description of the robot, visual representation of the robot, and the collision model of the robot.


**What do URDF files do?** It 3D models a robot or its parts, it simulate them or to simply help the developers in their daily work.

**How do I create a URDF file?** In a URDF File we must write the relationship between each **link** and **joint** in the robot and save the file with the *.urdf* extension

![images](https://user-images.githubusercontent.com/13907836/36333370-535776ca-132b-11e8-97e2-452b62057dc9.jpg)

![link](https://user-images.githubusercontent.com/13907836/36333893-6a448302-132e-11e8-978f-95726200bcc7.png)
![inertial](https://user-images.githubusercontent.com/13907836/36333896-6ae995e0-132e-11e8-8d3d-605cc050ade2.png)
![joint](https://user-images.githubusercontent.com/13907836/36333897-6b2a72cc-132e-11e8-8521-eadf98a53eca.png)

## Links
Links represents a single link of a robot. Using this, we can model a robot link and its properties. The modeling includes size, shape, color, and can even import a 3D mesh to represent the robot link. We can also provide dynamic properties of the link such as inertial matrix and collision properties.

XML CODE:                                                                     
```XML
<link name="<name of the link>">
  <inertial>...........</inertial>
  <visual> ............</visual>
  <collision>..........</collision>
</link>
```

Example:                                                                     
```XML
<link name="<forearm>">
<!-- *not needed for now* <inertial>...........</inertial> *This is a comment btw*-->
  <visual> 
    <geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box size="0.1 0.2 0.5"/>
    </geometry>
    <material name="Cyan">
      <color rgba="0 1.0 1.0 1.0"/>
    </material>
  </visual>
  <!--  *not needed for now* <collision>..........</collision> -->
</link>
```

In this example it represented a single link. 
 * The **Visual** section represents the real link of the robot. 
 * The **Collision** is the area surrounding the link. ( look at top pic for better understanding )
 * The **Inertial** is to get your model to simulate properly, you need to define several physical properties of your robot, i.e. the properties that a physics engine like Gazebo would need. *we will get back to this one*
 
 ## Joints
Joints represents a robot joint. We can specify the kinematics and dynamics of the joint and also set the limits of the joint movement and its velocity. The joint tag supports the different types of joints such as **revolute**, **continuous**, **prismatic**, **fixed**, **floating**, and **planar**. A URDF joint is formed between two links; the first is called the **Parent link** and the second is the **Child link**.


XML CODE:                                                                     
```XML
<joint name="<name of the joint>">
  <parent link="link1"/>
  <child link="link2"/>
  
  <calibration .... />
  <dynamics damping ..../>
  <limit effort .... />
</joint>
```

Example:                                                                     
```XML
<joint name="<forearm>">
  <parent link="forearm"/>
  <child link="gripper"/>
  <origin xyz="0.5 0.0 0.0" rpy="0 0 -1.57"/>
  <axis xyz="0 0 1"/>
  <!-- *not needed* <calibration .... /> -->
  <!-- *not needed* <dynamics damping ..../> -->
  <!-- *not needed* <limit effort .... /> -->
</joint>  
```
 ## Robot Tag name
 robot: This tag encapsulates the entire robot model that can be represented using URDF. Inside the robot tag, we can define the name of the robot, the links, and the joints of the robot.
 XML CODE:                                                                     
```XML
<robot name="<name of the robot>"
  <link>  ........ </link>
  <link>  ........ </link>
  <joint> ........ </joint>
  <joint> ........ </joint>
</robot>
```

![4ddrobot](https://user-images.githubusercontent.com/13907836/36337621-27144506-134f-11e8-9657-68b31921d472.PNG)

*this is robot visual 4dd_robot URDF;Use this [URDF Online Visualizer](http://www.mymodelrobot.appspot.com/5629499534213120) to see your URDF online*
Example:                                                                     
```XML
<?xml version='1.0'?>
<robot name="4dd_robot">
  
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="1 0.5 0.25" />
            </geometry>
            <material name="yellow">
                <color rgba="0.8 0.8 0 1" />
            </material>
        </visual>
    </link>
  
    <!-- Front Right Wheel -->
    <link name="f_r_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="1.570795 0 0" />
            <geometry>
                <cylinder length="0.1" radius="0.2" />
            </geometry>
            <material name="black">
                <color rgba="0.05 0.05 0.05 1" />
            </material>
        </visual>
    </link>
    <joint name="joint_f_r_wheel" type="continuous">
        <parent link="base_link" />
        <child link="f_r_wheel" />
        <origin xyz="0.25 -0.30 0" rpy="0 0 0" />
        <axis xyz="0 1 0" rpy="0 0 0" />
    </joint>
  
    <!-- Back Right Wheel -->
    <link name="b_r_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="1.570795 0 0" />
            <geometry>
                <cylinder length="0.1" radius="0.2" />
            </geometry>
            <material name="black" />
        </visual>
    </link>
    <joint name="joint_b_r_wheel" type="continuous">
        <parent link="base_link" />
        <child link="b_r_wheel" />
        <origin xyz="-0.25 -0.30 0" rpy="0 0 0" />
        <axis xyz="0 1 0" rpy="0 0 0" />
    </joint>
  
    <!-- Front Left Wheel -->
    <link name="f_l_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="1.570795 0 0" />
            <geometry>
                <cylinder length="0.1" radius="0.2" />
            </geometry>
            <material name="black" />
        </visual>
    </link>
    <joint name="joint_f_l_wheel" type="continuous">
        <parent link="base_link" />
        <child link="f_l_wheel" />
        <origin xyz="0.25 0.30 0" rpy="0 0 0" />
        <axis xyz="0 1 0" rpy="0 0 0" />
    </joint>
  
    <!-- Back Left Wheel -->
    <link name="b_l_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="1.570795 0 0" />
            <geometry>
                <cylinder length="0.1" radius="0.2" />
            </geometry>
            <material name="black" />
        </visual>
    </link>
    <joint name="joint_b_l_wheel" type="continuous">
        <parent link="base_link" />
        <child link="b_l_wheel" />
        <origin xyz="-0.25 0.30 0" rpy="0 0 0" />
        <axis xyz="0 1 0" rpy="0 0 0" />
    </joint>
</robot>
```
The ```<robot>``` tag defines the name of the robot that we are going to create. Here, we named the robot *4dd_robot*.
```XML
<?xml version='1.0'?>
<robot name="4dd_robot">
```
## Adding the URDF file to our ROS Package
within your ROS Package you can add a folder called "URDF" to put all your URDF files so they can be in your ROS workspace.

*Here is a tree view of my workspace.*

![screenshot from 2018-02-16 19-53-12](https://user-images.githubusercontent.com/13907836/36337791-c49eb0c8-1353-11e8-9cf7-a615c954417f.png)

To check if the URDF code has errors then using the following command in the termainal(*must be in the folder of the URDF file*):
```
lauro199471@lauro-PC:~/catkin_ws/src/ros_robotics/urdf$ check_urdf 4dd_robot.urdf 
```
The ```check_urdf``` command will parse urdf and show an error, if any. If everything
is OK, it will show an output as follows:

```
robot name is: 4dd_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 4 child(ren)
    child(1):  b_l_wheel
    child(2):  b_r_wheel
    child(3):  f_l_wheel
    child(4):  f_r_wheel
```

If we want to view the structure of the robot links and joints graphically, we can use
a command tool called ```urdf_to_graphiz``` :
```
lauro199471@lauro-PC:~/catkin_ws/src/ros_robotics/urdf$ urdf_to_graphiz 4dd_robot.urdf 
```
This command will generate two files: 4dd_robot.gv and 4dd_robot.pdf.(*GV file is a document which includes descriptions about graphs and written using the DOT Language. DOT Language is a simple text graph language that is used to interpret abstract representation of objects that is usually joined by links*)

*This is what the 4dd_robot.gv and 4dd_robot.pdf look like*
![screenshot-from-2018-02-16-20-](https://user-images.githubusercontent.com/13907836/36337885-e831eda0-1355-11e8-878d-3835293afcd7.png) 


## How to simulate URDF on RVIZ
To simulate URDF on RVIZ we must first understand what a *joint state publisher node* and a *Robot State node*.Rviz, abbreviation for ROS visualization, is a powerful 3D visualization tool for ROS. It allows the user to view the robot model, display and/or log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions. 

## Understanding Joint State Publisher
Joint state publisher is one of the ROS packages that is commonly used to interact with each joint of the robot. The package contains the joint_state_publisher node, which will find the nonfixed joints from the URDF model and publish the joint state values of each joint in the **sensor_msgs/JointState** message format. To display a slider based control window to control each joint we will have to set *use_gui* to true.
```XML
<!-- This is in the launch file -->
<param name="use_gui" value="true"/>
```
The lower and upper value of a joint will be taken from the lower and upper values associated with the *limit tag* used inside the *joint tag.*
Example: 
```XML
<!-- Joint between Base Link and Middle Link -->
  <joint name="joint_base_mid" type="revolute">
    <parent link="base_link"/>
    <child link="mid_link"/>
    <origin xyz="0 ${width} ${height1 - axle_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="${damp}"/>
    <limit effort="100.0" velocity="0.5" lower="-3.14" upper="3.14" />
</joint>
```
## Understanding Robot State Publisher 
The *robot state publisher package* helps to publish the state of the robot to *tf*. This package subscribes to joint states of the robot and publishes the 3D pose of each link using the kinematic representation from the URDF model.

## Launch File for URDF and Simulate in RVIZ 
Using URDF we create a launch file to simulate the URDF file in RVIZ. Here is an example launch file using the URDF file from above.
```XML
<launch>
  <param name="use_gui" value="true"/>
  <!-- set these parameters on Parameter Server -->
  <param name="robot_description" textfile="$(find ros_robotics)/urdf/4dd_robot.urdf" />
   <!-- Start 3 nodes: joint_state_publisher, robot_state_publisher and rviz -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" required="true" />
</launch>
```
This is how it should look in RVIZ with a joint_state_publisher scroll bar to move the joints. In this case the joints are the wheels.

![screenshot from 2018-02-19 10-57-00](https://user-images.githubusercontent.com/13907836/36393413-4d7a39e6-1564-11e8-8d48-87472acd2dd3.png)

## Adding physical and collision properties to a URDF model 
Before simulating a robot in a robot simulator, such as Gazebo, V-REP, and so on, we need to define the robot link's physical properties such as geometry, color, mass, and inertia, and the collision properties of the link.We will only get good simulation results if we define all these properties inside the robot model. URDF provides tags to include all these parameters and code snippets of base_link contained in theses properties as given here:
```XML
<link> 
......    
  <collision>      
      <geometry>      
          <cylinder length="0.03" radius="0.2"/>      
      </geometry>      
      <origin rpy="0 0 0" xyz="0 0 0"/>
  </collision>
    
  <inertial>    
      <mass value="1"/>    
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>     
  </inertial> 
........... 
</link> 
```
Here, we define the collision geometry as cylinder and the mass as 1 Kg, and we also set the inertial matrix of the link. The **collision and inertia parameters** are required in each link; otherwise, Gazebo will not load the robot model properly.
### 1. Collision Tag
The collision element defines its shape the same way the visual element does, with a geometry tag. The format for the geometry tag is exactly the same here as with the visual.You can also specify an origin in the same way as a subelement of the collision tag (as with the visual). 

In many cases, you’ll want the collision geometry and origin to be exactly the same as the visual geometry and origin. 

### 2. Physical Properties
In order to get your model to simulate properly, you need to define several physical properties of your robot, i.e. the properties that a physics engine like Gazebo would need. The 3x3 rotational inertia matrix is specified with the inertia element.
#### 2.1 Inertia
Every link element being simulated needs an inertial tag.The mass is defined in kilograms. To calulate moment of interia use this tool:
**(MAKE TOOL)***

## Gazebo
Gazebo is a free and open source robot simulation environment developed by Willow Garage.To run Gazebo requires a powerful graphics card. Roslaunch is a standard method used to start Gazebo with world files and robot URDF models. To perform a basic test of Gazebo, an empty Gazebo world can be brought up with the following command:
```
$ roslaunch gazebo_ros empty_world.launch 
```
## Modifications to the robot URDF 
Gazebo expects the robot model file to be in SDF format. SDF is similar to the URDF, using some of the same XML descriptive tags. With the following modifications, Gazebo will automatically convert the URDF code into an SDF robot description. The following sections will describe the steps to be taken.
### Adding the Gazebo tag
The ```<gazebo>``` tag must be added to the URDF to specify additional elements needed for simulation in Gazebo. This tag allows for identifying elements found in the SDF format that are not found in the URDF format. If a ```<gazebo>``` tag is used without a reference="" property, it is assumed that the <gazebo> elements refer to the whole robot model. The reference parameter usually refers to a specific robot link. 

### Specifying color in Gazebo 
The method of specifying link colors in rviz does not work in Gazebo since Gazebo has adopted OGRE's material scripts for coloring and texturing links. Therefore, a Gazebo ```<material>``` tag must be specified for each link. These tags can be placed in the model file just before the ending ```</robot>``` tag:

Example:
```XML
<gazebo reference="base_link">
    <material>Gazebo/Blue</material>
</gazebo>
<gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
</gazebo>
<gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
</gazebo>
``` 
## URDF with Xacro
Xacro is a very simple language that allows us to create URDF files using macros that can contain simple instructions and basic math. The main advantage of using xacro is that we can take advantage of the iterative nature of robot links by defining them as macros that get repeated with different parameters throughout the robot. Using this approach saves time, increases readability, and is less error-prone.Using xacro, we can declare constants or properties that are the named values inside the xacro file, which can be used anywhere in the code. The main use of these constant definitions are, instead of giving hard coded values on links and joints, we can keep constants like this and it will be easier to change these values rather than finding the hard coded values and replacing them. The differences for the Xacro format are listed here and explained in more detail after the code is presented: 
  * Addition of the XML namespace declaration on the second line 
  * Use of the Xacro ```<property>``` tag to define constant values 
  * Addition of property names instead of values within the ```<box>`` and ``<origin>``` tags 
  * Simple math (along with property names) to calculate link ```<origin>``` z values
  * 
Ex of using ```<property>``` for constants:
```XML
 <!-- Constants for robot dimensions(values are in meters fyi) -->
 <xacro:property name="width" value="0.1" />  
 <xacro:property name="height1" value="2" />
 <xacro:property name="height2" value="1" />  
 <xacro:property name="radius" value="0.04" /> 
```
To use these 'names' we put ```${(name of property name)}``` which is evaluated as the value of the ``` name of protery name```.
Ex:
```XML
 <xacro:property name="width" value="0.1" />  <!-- ${width} = 0.1 -->
```
In order to create the URDF file from Xacro files, the Xacro file must contain an XML namespace declaration using the ```xmlns``` attribute with the xacro tag and corresponding URI. Here is the XML namespace (xmlns) attribute for our 4dd_robot:
```XML
 <!-- This declaration is vital for the file to parse properly. -->
 <robot name="4dd_robot" xmlns:xacro="http://www.ros.org/wiki/xacro"> 
```
 The main feature of Xacro is **Marcos**. When creating a macro, a simple ```<xacro>``` tag can expand into a statement or sequence of statements in the URDF/SDF file. Macros are extremely useful when statements are repeated or reused with modifications defined by parameters. 
Ex:
```XML
<?xml version="1.0"?>
<!-- THIS IS NECESSARY -->
<robot name="4dd_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!--//////////////////////////////////////////////////////////////////////////// -->
    <!-- Defining the colors used in this robot -->
    <material name="Yellow">
        <color rgba="0.8 0.8 0 1" />
    </material>
    <material name="Black">
        <color rgba="0.05 0.05 0.05 1" />
    </material>
    <!--//////////////////////////////////////////////////////////////////////////// -->
    <!-- Macro for calculating inertia of cylinder -->
    <macro name="cylinder_inertia" params="m r h">
        <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0" iyx="0" iyy="${m*(3*r*r+h*h)/12}" iyz="0" izx="0" izy="0" izz="${m*r*r/2}" />
    </macro>
    <!-- Macro for calculating inertia of cube -->
    <macro name="cube_inertia" params="w h d m">
        <inertia ixx="${m*(d*d+h*h)/12}" ixy="0" ixz="0" iyx="0" iyy="${m*(w*w+d*d)/12}" iyz="0" izx="0" izy="0" izz="${m*(w*w+h*h)/12}" />
    </macro>
    <!--//////////////////////////////////////////////////////////////////////////// -->
    <!-- BASE LINK -->
    <property name="base_mass" value="5" />
    <!-- in kg-->
    <property name="base_depth" value="1" />
    <!-- in m-->
    <property name="base_height" value="0.25" />
    <!-- in m-->
    <property name="base_width" value="0.5" />
    <!-- in m-->
    <!--Actual body/chassis of the robot-->
    <link name="base_link">
        <!-- 1) Add Interia for Base -->
        <inertial>
            <mass value="${base_mass}" />
            <origin xyz="0 0 0" />
            <cube_inertia m="${base_mass}" d="${base_depth}" h="${base_height}" w="${base_width}" />
        </inertial>
        <!-- 2) Add Visual for Base -->
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="${base_depth} ${base_width} ${base_height}" />
                <!--length(depth) x width x height -->
            </geometry>
            <material name="Yellow" />
        </visual>
        <!-- 3) Add Collision for Base -->
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0 " />
            <geometry>
                <box size="${base_depth} ${base_width} ${base_height} " />
                <!--length(depth) x width x height -->
            </geometry>
        </collision>
    </link>
    <!-- FRONT RIGHT WHEEL  -->
    <!--Actual body/chassis of the robot-->
    <link name="f_r_wheel">
        <!-- 1) Add Interia for Base -->
        <inertial>
            <mass value="${base_mass}" />
            <origin xyz="0 0 0" />
            <cube_inertia m="${base_mass}" d="${base_depth}" h="${base_height}" w="${base_width}" />
        </inertial>
        <!-- 2) Add Visual for Base -->
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="${base_depth} ${base_width} ${base_height}" />
                <!--length(depth) x width x height -->
            </geometry>
            <material name="Yellow" />
        </visual>
        <!-- 3) Add Collision for Base -->
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0 " />
            <geometry>
                <box size="${base_depth} ${base_width} ${base_height} " />
                <!--length(depth) x width x height -->
            </geometry>
        </collision>
    </link>
</robot>
```
## 7 DOF Arm using Xarco
![capture](https://user-images.githubusercontent.com/13907836/36492546-3df7322e-16e2-11e8-8d25-c4120dab8229.PNG)
Let's start creating the seven DOF arm; the final output model of the robot arm is shown above.
![7dof](https://user-images.githubusercontent.com/13907836/36493898-7b62f80c-16e5-11e8-82c1-67637b277694.PNG)
