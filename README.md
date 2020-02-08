# Non-Linear-to-Linear-system
Design an output feedback controller for a non-linear system by linearization, and checking for observability and controllability. 

%
This is a Controls project divided into two main components.

First is is obtain the equation of motion for a non-linearized system , write state space of the linearized sytems at equilibrium points,
and obtain conditions for which the system is controllable. For that controllable system, design a LQR controller for the system and use 
Lyapunov function to certify its stability.

Second component is the output component. For a given output vector, find the vector for which the linearized system is observable, obtain 
the best Luenberger observer for each vector and design an output feedback controller (LQG)
