
# Flying Car Nanodegree Control Project

For this project in Udacity's Flying Car Nanodegree we learn about the physics of a quadcopter in order to control it. We learn to model the vehicle and provide instructions for the quadcopter to follow a desired trajectory. I implemented a 5 piece Cascaded controller that I tuned both manually and with gradient ascent a.k.a. the twiddle algorithmn.

In order to control the quadcopter, I needed to model its ability to travel in the x, y, and z planes as well as its roll, pitch and yaw as indicated with p, q, and r below. Then with the model predict where the quadcopter would travel and implement controllers to make it happen. 
<p align="center">
<img src="animations/xyz.png" width="500"/>
</p>
What follows is a brief walkthrough of what I did to pass this project. To follow along with the simulator, the starter code is located [here](https://github.com/udacity/FCND-Controls-CPP). 

## Controllers

Cascaded controllers work together to follow the path sent to them from the planner. The controllers are layered and the inner most layers are fed information from the outer layers. In our case, the planner tells the controllers that we are to go to x, y, z position in the world frame. The altitude and lateral controllers then take that information and some state information to tell the roll-pitch and yaw controllers what acceleration is needed to get there, which makes the roll-pitch and yaw controllers calculate how the quadcopter needs to be oriented to acheive that acceleration. Finally with the predicted change in orientation the body rate controllers predict how much torque is needed to bring the quadcopter to align itself. The over-all force and the moments (torques) are fed to the quadcopter for it to move.

The following is a picture of the controllers and how they feed into one another.
<p align="center">
<img src="animations/cascaded controller.png" width="800"/>
</p>

When implementing controllers, it is helpful to tune the parameters for the faster most inner leveled controllers, so that when the faster controllers have told the quadcopter to move by given amounts it has arrived to the desired position by the time the outer levels are calculating the next steps.

### Body Rate Controller
So starting with the body rate controller, I implemented a proportional controller that takes the desired body rates in radians per second and the current radians per second to output the moments or torque for each of the three axes. The code for this is located in the QuadControll.cpp file in the BodyRateControl function.

### Roll Pitch Controller
Then I implemented the roll pitch controller that uses the acceleration and thrust commands from the altitude and lateral controls to output a body rate command. The controller accounts for the non-linear transformation from local accelerations to body rates and accounts for the drone's mass. The following equations are used to calculate the desired pitch and roll rates.

b&#775;<sup>x</sup><sub>c</sub> = k<sub>p</sub>(b<sup>x</sup><sub>c</sub> - b<sup>x</sup><sub>a</sub>)

b&#775;<sup>y</sup><sub>c</sub> = k<sub>p</sub>(b<sup>y</sup><sub>c</sub> - b<sup>y</sup><sub>a</sub>)

where b<sup>x</sup><sub>a</sub> = R<sub>13</sub> and b<sup>y</sup><sub>a</sub> = R<sub>23</sub> from the rotation matrix R. The given values can be converted into the angular velocities into the body frame by the next matrix multiplication.
<p align="center">
<img src="animations/matrix rollPitch.png" width="300"/>
</p>

Also don't forget to convert the force from Newtons to acceleration with the mass like this `commanded thrust / mass`. See the code in the RollPitchControl function.

### Altitude Controller
Next I implemented the altitude controller because it is critical in preventing the quadcontroller from crashing into the ground. For this one I implemented a PID controller. This is essential because if the quadcopter's mass changes or is wrong in the config file the quadcopter could crash into the ground or go too high. If you only have a P or PD controller it won't correct itself properly. The scenario 4 in the simulator has one drone that is significantly heavier than the rest and without the Integral portion it falls to the ground quickly.

Using the position error (P), the acceleration error (I) and the velocity error (D), I calculated the error and used it in the following code:

```  
thrust = (error - gravity) / R(2, 2)
thrust = -mass * CONSTRAIN(thrust, -maxDescentRate/dt, maxAscentRate/dt);
```

The R(2,2) accounts for the non-linear effects of having the quadcopter rotated along the roll and pitch angles. Then since the z axis is pointed down we use the -mass * the thrust within the bounds of the max and min rates of the quadcopter over how much time has passed.

### Lateral Position Controller and the Yaw Controller

The lateral position controller takes the desired position and velocity and the current state and outputs the needed horizontal accelerations. It is a pretty generic controller, but I had to make sure to include the max and min speed and acceleration rates. When I didn't the quadcopter behaved funny because it wasn't able to follow what the controllers were saying to do due to its inability to go faster than the max.

Similarly the yaw controller is a simple controller with the error being restricted between [0,b]. I did this by adding or subtracting 2*PI if the error was less than negative PI or greater than positive PI.

### Moment Command

Now putting together the four propellers, they each exhert force, and increasing the force of each one in different patterns gives the quadcopter its ability to roll, pitch, and yaw. Therefore we control them individually, but the following equations allow us to calculate our x, y, and z moments.

<p align="center">
<img src="animations/momentcommand.png" width="300"/>
</p>
The force from the two motors on the left (F<sub>1</sub> and F<sub>4</sub>) counter the forces from the two on the right (F<sub>2</sub> and F<sub>3</sub>) to give the Tau<sub>x</sub> or roll.
<p></p>
In the equation `l` is the arm length from the center to the propeller which we multiply by `1/sqrt(2)`, and 𝜔 is the torque of the propeller. The 𝜏<sub>1</sub>=−𝑘<sub>𝑚</sub>𝜔<sup>2</sup><sub>1</sub> , 𝜏<sub>2</sub>=−𝑘<sub>𝑚</sub>𝜔<sup>2</sup><sub>2</sub> , 𝜏<sub>3</sub>=−𝑘<sub>𝑚</sub>𝜔<sup>2</sup><sub>3</sub>,
𝜏<sub>4</sub>=−𝑘<sub>𝑚</sub>𝜔<sup>2</sup><sub>4</sub>
<p></p>
This next part was confusing to me at first because the lectures use the above equations and then in the project they expect you to know or use another derivation of equations. 
<p></p> 
The trick is tau = kappa * F which means you can re-arange the formulas to get:<br>
𝜏<sub>x</sub> = (F1 - F2 - F3 + F4)*l<br>
𝜏<sub>y</sub> = (F1 + F2 - F3 - F4)*l<br>
𝜏<sub>z</sub> = (F1 - F2 + F3 - F4)*kappa<br>

Since we know total thrust F = F1 + F2 + F3 + F4 then we can use these four equations to solve the linear system giving:<br>

F1 = 1/4 * (F + 𝜏<sub>x</sub> / l + 𝜏<sub>y</sub> / l + 𝜏<sub>z</sub> / kappa)<br>
F2 = 1/4 * (F - 𝜏<sub>x</sub> / l + 𝜏<sub>y</sub> / l - 𝜏<sub>z</sub> / kappa)<br>
F3 = 1/4 * (F - 𝜏<sub>x</sub> / l - 𝜏<sub>y</sub> / l + 𝜏<sub>z</sub> / kappa)<br>
F4 = 1/4 * (F + 𝜏<sub>x</sub> / l - 𝜏<sub>y</sub> / l - 𝜏<sub>z</sub> / kappa)<br>

I hope this makes sense, but using these equations you can take the desired collective thrust given for the x,y,z planes which in my code is the momentCmd variable and put it into the above equation. Here is the code:

```
    //collThrustCmd: desired collective thrust [N]
    //momentCmd: desired rotation moment about each axis [N m]

	float d_perp = L * static_cast<float>(M_SQRT1_2);
	float c_bar = collThrustCmd; 
	float p_bar = momentCmd.x / d_perp;
	float q_bar = momentCmd.y / d_perp;
	float r_bar = - momentCmd.z / kappa;

	float f1 = 0.25 * (c_bar + p_bar + q_bar + r_bar);
	float f2 = 0.25 * (c_bar - p_bar + q_bar - r_bar);
	float f3 = 0.25 * (c_bar + p_bar - q_bar - r_bar);
	float f4 = 0.25 * (c_bar - p_bar - q_bar + r_bar);
  
    cmd.desiredThrustsN[0] = f1; // front left
	cmd.desiredThrustsN[1] = f2; // front right
	cmd.desiredThrustsN[2] = f3; // rear left
	cmd.desiredThrustsN[3] = f4; // rear right
```

This fulfills the writeup for my project, but please look forward to me updating it in greater detail in a blog post.
