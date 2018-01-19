## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)
[image0]: ./misc_images/links.jpg
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/image4.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I found the link lengths from the kr210.urdf.xacro file, but I was absolutely mystified as to how the link offsets are derived. This video helped: https://www.youtube.com/watch?v=rA9tm0gTln8 although I still have NO idea as to where the 'd' values come from. In the interest of time, I took the values from the walkthrough video.
![alt text][image4]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
```
    Individual transformation matrices
    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
    T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```
T0_1:
```
       [ 1.  , -0.  ,  0.  ,  0.  ]
       [ 0.  ,  1.  , -0.  , -0.  ]
       [ 0.  ,  0.  ,  1.  ,  0.75]
       [ 0.  ,  0.  ,  0.  ,  1.  ]
```
T1_2:
```
       [  6.12323400e-17,   1.00000000e+00,   0.00000000e+00, 3.50000000e-01]
       [ -6.12323400e-17,   3.74939946e-33,   1.00000000e+00, 0.00000000e+00]
       [  1.00000000e+00,  -6.12323400e-17,   6.12323400e-17, 0.00000000e+00]
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]
```
T2_3:
```
       [ 1.  , -0.  ,  0.  ,  1.25]
       [ 0.  ,  1.  , -0.  , -0.  ]
       [ 0.  ,  0.  ,  1.  ,  0.  ]
       [ 0.  ,  0.  ,  0.  ,  1.  ]
```
T3_4:
```
       [  1.00000000e+00,  -0.00000000e+00,   0.00000000e+00, -5.40000000e-02]
       [  0.00000000e+00,   6.12323400e-17,   1.00000000e+00, 1.50000000e+00]
       [ -0.00000000e+00,  -1.00000000e+00,   6.12323400e-17, 9.18485099e-17]
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]
```
T4_5:
```
       [  1.00000000e+00,  -0.00000000e+00,   0.00000000e+00, 0.00000000e+00]
       [  0.00000000e+00,   6.12323400e-17,  -1.00000000e+00, -0.00000000e+00]
       [  0.00000000e+00,   1.00000000e+00,   6.12323400e-17, 0.00000000e+00]
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]
```
T5_6:
```
       [  1.00000000e+00,  -0.00000000e+00,   0.00000000e+00, 0.00000000e+00]
       [  0.00000000e+00,   6.12323400e-17,   1.00000000e+00, 0.00000000e+00]
       [ -0.00000000e+00,  -1.00000000e+00,   6.12323400e-17, 0.00000000e+00]
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]
```
T6_EE:
```
       [ 1.   , -0.   ,  0.   ,  0.   ]
       [ 0.   ,  1.   , -0.   , -0.   ]
       [ 0.   ,  0.   ,  1.   ,  0.303]
       [ 0.   ,  0.   ,  0.   ,  1.   ]
```
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # EE rotation matrix
    # reference http://planning.cs.uiuc.edu/node102.html

    r, p, y = symbols('r p y')

    ROT_x = Matrix([[1, 0, 0],
            [0, cos(r), -sin(r)],
            [0, sin(r), cos(r)]]) #ROLL

    ROT_y = Matrix([[cos(p),   0, sin(p)],
            [0,        1,  0],
            [-sin(p),  0, cos(p)]]) #PITCH


    ROT_z = Matrix([[cos(y),   -sin(y), 0],
            [sin(y),    cos(y), 0],
            [0,         0,      1]])  #YAW

    ROT_EE = ROT_z * ROT_y * ROT_x
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.


![alt text][image0]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.



