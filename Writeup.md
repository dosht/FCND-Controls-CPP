# Project: Building a Controller

I started by implementing the python controller [link](https://github.com/dosht/FCND-Controls/blob/master/controls_flyer.py)
and then moved all control code to the C++ controller. I tested the controlers in this sequence: (motoros commands, body rate controler and roll-pitch controller), then yaw controller, then altitude contoller and lateral controller, and then I added the integeral term to the altitude contoller to mass error in scnario 4, and finally I spend good time tuning parameteres and test between all scenarios. I found when I keep the ratio between related parameters, it could converge easily to the good set of parameters e.g. `kpPosZ, kdPosZ, kiPosZ` or `kpPosXY, kdPosXY`. The lowers level params like `kpBank` and `kpPQR`, I didn't need to tune a lot after passing the attitude scenario.

### Controller Video

[![Controller Video](http://img.youtube.com/vi/HWr-DwxJvTk/0.jpg)](http://www.youtube.com/watch?v=HWr-DwxJvTk)


## Implemented body rate control in C++
![body rate equation](/images/body-rate-ctrl.gif)

First, I calculate the error between current body rate and target body rate and then multiply by `pqr` gain and 
by moment of inertia to get the moment.

```C++
V3F err = pqrCmd - pqr;
V3F u_ = err * kpPQR;
momentCmd = u_ * Ixyz;
```

## Implement roll pitch control in C++
![roll pitch equation](/images/roll-pitch-ctrl.gif)

I used this equation from this paper: [Feed-Forward Parameter Identification for Precise Periodic
Quadrocopter Motions](http://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).

![rollPitchCtrl.gif](/images/rollPitchCtrl.gif)

First, we need to convert current attitude from world frame into body frame by creating the rotation matrix.
Second, convert commanded collective thrust (N) into acceleration (m/s2) by dividing on mass, then 
calculate the error and multiply it be kpBank. Finally, from the equation above, we can get the desired `p,q,r`
for roll and pitch component that will be passed to body rate controller.

## Implement altitude controller in C++
![altitude equation](/images/altitude-ctrl.gif)

First, I calculated `pTerm` and `dTerm` the position error `posZCmd - posZ` and velocity error `velZCmd - velZ` and multiply by `kpPosZ`
and `kpVelZ` respectively, then I needed to calculate also integrated altitude error `iTerm` by accumulating the 
error over multiple calls to correct the difference between the mass parameter and real drone mass.

Then, I calculated `u1Bar` by adding `pTerm`, `dTerm` and 'iTerm` and `accelZCmd` to consider the target accelelration.
Finally, calculated the required thrust `-(u1 - g) / m` and divide by `bz` (R33) from the rotation matrix.

## Implement lateral position control in C++
![lateral equation](/images/lateral-ctrl.gif)

This is a high level controller that decides the required lateral acceleration in (X, Y) dimensions,
and only calculate the error between target/current position and target/current velocity and multibly by 
control gains `kpPosXY, kpPosXY` and `kpVelXY, kpVelXY` and add the target acceleration.
Also, I should bound the velocity and acceleration between the feasible min/max velocity and acceleration.

## Implement yaw control in C++
![yaw equation](/images/yaw-ctrl.gif)

Just calculate the error between current yaw position and target yaw and multiply by `kpWay`. But also, there is
an important step which is commanded yaw rate -π and π and set the right direction.

## Implement calculating the motor commands given commanded thrust and moments in C++
![motors commands equation](/images/motors-commands.gif)

This is the lowest level that calculate the thrust (Neuton) for each rotor from the collective commanded thrust
(Neuton) and commanded moment in body frame. We are doing this by solving these equations:

![motors foreces moment equation](/images/forces-moment-equation.gif)

```
F: Forces of the 4 rotors.
M: moment in the 3 axis.
```
