clear;
clc;
%% Constants

%Rotational Spring and damping
ks = 0.0015;
kd = 0.0005;
%Lengths(l1 = beam to E, L2 = beam to Phi)
l1 = 0.110;
l2 = 0.070;
%Thrust input(Gain = ka,a;b = characteristics)
a = 0.0028;
b = -0.0057;
ka = 1.2;
%Equilibrium input voltage
Ue = 5.5;
%Interia(E,Phi,Theta)
JE = 0.0024;  
JPhi = 0.000495;
JTh = 0.0024;

%% Linearised A B C D Matrices 

A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, (-ks/JPhi), 0, 0, (-kd/JPhi), 0;
     0, (2*l1*b/JE), 0, 0, 0, 0];
 
 B = [0, 0;
      0, 0;
      0, 0;
      (l1*a*ka/JE), (l1*a*ka/JE);
      (l2*a*ka/JPhi), (-l2*a*ka/JPhi);
      0, 0];
  
C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];
     
D = zeros(3,2);

 %% Stability
 
OL_poles = eig(A);
% No Positive real poles
% Therefore is asmtotically stable about 
% equilibrium point

%% Controllability/ Reachability 

Cont = ctrb(A,B);
Is_Cont = rank(Cont);
% The controllability matrix is full rank
% Therefore, the system is fully controllable

%% Observablity

Obsv = obsv(A,C);
Is_Obsv = rank(Obsv);
% The observabilty matrix is full rank
% Therefore, the system is fully observable
