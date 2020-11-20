# Quadcopter Dynamics

> Introduction : Quadcopter, also known as quadrotor, is a UAV with four rotors. The rotors
are directed upwards and they are placed in a Cross formation with equal distance
from the center of mass of the quadcopter. The quadcopter is controlled by adjusting
the angular velocities of the rotors which are spun by electrical DC motors. Quadcopter
is a typical design for small unmanned aerial vehicles (UAV) because of the simple
structure. Quadcopters are used in surveillance, search and rescue, construction
inspections and several other applications.

###### Body Frame 

##### Body Frame It is fixed to the body which is being analyzed. The axis can be oriented arbitrarily, but are generally chosen to be the principal axes of inertia. The body (to which the body frame is fixed) as observed with respect to the body frame is stationary.

###### Inertial Frame 

##### An inertial frame of reference can be defined in analytical terms as a frame of reference that describes time and space homogeneously, isotropically, and in a time-independent manner. Conceptually, the physics of a system in an inertial frame have no causes external to the system. 

<p align="center">
  <img width="250" src="https://i0.wp.com/www.mdpi.com/applsci/applsci-09-03873/article_deploy/html/images/applsci-09-03873-g001.png">
</p>

###### The Rotational Matrix from the body frame to the inertial frame can be written as :

         [CψCθ CψSθSφ − SψCφ CψSθCφ + SψSφ]
    R =  [SψCθ SψSθSφ + CψCφ SψSθCφ − CψSφ]
         [−Sθ           CθSφ          CθCφ]

###### The rotation matrix R is orthogonal thus, 
###### <p>R<sup>-1</sup> = R <sup>T</sup></p>  
###### which is the rotation matrix from the inertial frame to the body frame.

###### The Transformation Matrix of Angular Velocities from body to inertial frame and viceversa are written as below where [φ* θ* ψ*] are angular velocites in the body frame and [p q r] are angular velocities in inertial frame. 

         [φ*]   [1  SφTθ   CφT ][p]
         [θ*] = [0  Cφ     −Sφ ][q]
         [ψ*]   [0  Sφ/Cθ Cφ/Cθ][r]
         
         [p]   [1   0  −Sθ ][φ*]
         [q] = [0  Cφ  CθSφ][θ*]
         [r]   [0 −Sφ  CθCφ][ψ*]
         
In the code the Aerodynamic drag is considered where Ax, Ay, Az are the drag force coefficients for velocities in the corresponding directions of the inertial frame. 

         
