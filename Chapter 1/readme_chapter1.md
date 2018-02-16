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

## Links
Links represents a single link of a robot. Using this, we can model a robot link and its properties. The modeling includes size, shape, color, and can even import a 3D mesh to represent the robot link. We can also provide dynamic properties of the link such as inertial matrix and collision properties.
