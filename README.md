# Quadcopter Dynamics

### Table of Contents 
- `Introduction`

- `Body Frame`

- `Inertial Frame`

- `Rotational Matrix`

- `Angular Velocities`

- `Newton - Euler Equations`

- `Code`

- `Result`

- `Conclusion`


#### Introduction

Quadcopter, also known as quadrotor, is a UAV with four rotors. The rotors
are directed upwards and they are placed in a Cross formation with equal distance
from the center of mass of the quadcopter. The quadcopter is controlled by adjusting
the angular velocities of the rotors which are spun by electrical DC motors. Quadcopter
is a typical design for small unmanned aerial vehicles (UAV) because of the simple
structure. Quadcopters are used in surveillance, search and rescue, construction
inspections and several other applications.

#### Body Frame
A Body Fame is fixed to the body which is being analyzed. The axis can be oriented arbitrarily, but are generally chosen to be the principal axes of inertia. The body (to which the body frame is fixed) as observed with respect to the body frame is stationary.

#### Inertial Frame 
An inertial frame of reference can be defined in analytical terms as a frame of reference that describes time and space homogeneously, isotropically, and in a time-independent manner. Conceptually, the physics of a system in an inertial frame have no causes external to the system. 

<p align="center">
  <img width="250" src="https://i0.wp.com/www.mdpi.com/applsci/applsci-09-03873/article_deploy/html/images/applsci-09-03873-g001.png">
</p>

###### The Rotational Matrix from the body frame to the inertial frame can be written as :

         [CψCθ CψSθSφ − SψCφ CψSθCφ + SψSφ]
    R =  [SψCθ SψSθSφ + CψCφ SψSθCφ − CψSφ]
         [−Sθ           CθSφ          CθCφ]

> The rotation matrix R is orthogonal thus, 
> <p>R<sup>-1</sup> = R <sup>T</sup></p>   
> which is the rotation matrix from the inertial frame to the body frame.

#### Angular Velocities 
The Transformation Matrix of Angular Velocities from body to inertial frame and viceversa are written as below where [φ* θ* ψ*] are angular velocites in the body frame and [p q r] are angular velocities in inertial frame. 

         [φ*]   [1  SφTθ   CφT ][p]
         [θ*] = [0  Cφ     −Sφ ][q]
         [ψ*]   [0  Sφ/Cθ Cφ/Cθ][r]
         
         [p]   [1   0  −Sθ ][φ*]
         [q] = [0  Cφ  CθSφ][θ*]
         [r]   [0 −Sφ  CθCφ][ψ*]
         
#### Newton - Euler Equations
In the code the Aerodynamic drag is considered where Ax, Ay, Az are the drag force coefficients for velocities in the corresponding directions of the inertial frame. After considering the drag the linear acceleration will be caluculated as follows : 

         [x**]      [0]       [CψSθCφ + SψSφ]       [Ax 0 0] [x*]
         [y**] = -g [0] + T/m [SψSθCφ − CψSφ] - 1/m [0 Ay 0] [y*]
         [z**]      [1]       [    CθCφ     ]       [0 0 Az] [z*]
         
         // where x**, y**, z** are linear acceleration in xyz directions and x*, y*, z* are the linear velocities...
The angular accelerations (p*, q*, r*) can be formulated as :
          
            [p*]   [     L(F2 - F4)    ]   [p]     [p]
        (I) [q*] = [     L(F3 - F1)    ] - [q] x I [q]
            [r*]   [ M1 - M2 + M3 - M4 ]   [r]     [r]
          
#### Code 

In the code, the Thrust is been caluculated as
          
       T = (g + Kd ( zd* − z*) + Kp (zd − z)) m/CφCθ
       
       // Where zd = desired state of z coordinate and zd* is the desired linear velocity 
       // The above equation is considered as a PD controller
       
Similarly, Torques for roll pitch and yaw are τφ, τθ, τψ
         
        τφ = (Kd(φd* − φ*)+ Kp(φd − φ))Ixx,
        τθ = (Kd(θd* − θ*)+ Kp(θd − θ))Iyy,
        τψ = (Kd(ψd* − ψ*)+ Kp(ψd − ψ))Izz.
        
In the above equations gravity g, and mass m and moments of inertia I of the quadcopter are considered.
The correct angular velocities of rotors ωi(controlsignal to the DC motors) and (i = 1, 2, 3, 4) can be calculated as : 

        controlsignal(1) = T/4k − τθ/2kl − τψ/4b
        controlsignal(2) = T/4k − τφ/2kl + τψ/4b
        controlsignal(3) = T/4k + τθ/2kl − τψ/4b
        controlsignal(4) = T/4k + τφ/2kl + τψ/4b

#### Result - Linear Displacement and Angular Displacement 

<p align="center">
  <img width="500" src="https://user-images.githubusercontent.com/69350191/99878243-0fc8d280-2c2a-11eb-96b8-4dcb77c2242e.PNG">
</p>

       // Here the desired state.z is set to 10 unit. Hence we can observe that the blue line stablizes at 10 or ~10 units from 5 sec and hovers at the desired point. 
       // Initially [θ φ φ] are set at 45 degrees and are desired to be at 0 degree after reaching the setpoint... 
       
#### Conclusion 

The mathematical model of quadcopter dynamics was presented and the differential equations
were derived from the Newton-Euler and the Euler-Lagrange equations. The model
was verified by simulating the flight of a quadcopter with Matlab. Stabilisation of
attitude of the quadcopter was done by utilising a PD controller. 
