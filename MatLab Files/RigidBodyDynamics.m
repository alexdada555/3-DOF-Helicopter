clear;
clc;
%% Constants

%Rotational Spring and damping
ks = 0.0015;           % Spring constant (kg*m^2*s^-2)
kd = 0.0005;           % Viscous damping (kg*m^2*s^-1)
m1  = 0.0505;   % mass of fan assembly (kg)
m2  = 0.100;    % mass of counterweight (kg)
%Lengths(l1 = beam to E, L2 = beam to Phi)
l1  = 0.110;    % distance from helicopter arm to elevation axis (m);
l2  = 0.070;    % distance from fan centres to pitch axis (m);
l3  = 0.108;    % distance from counterweight to elevation axis (m);
%Thrust input(Gain = ka,a;b = characteristics)
a = 0.0045;
b = -0.016;
ka = 1.2;
%Equilibrium input voltage
Ue = 5.3;
%Interia(E,Phi,Theta)
JE  = 2*m1*(l1^2)+m2*(l3^2);    % Inertia about elevation axis (kg*m^2);
JTh  = JE;                       % Travel axis inertia
JPhi  = 2*m1*(l2^2);              % Pitch axis inertia

%% Linearised A B C D Matrices 

A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, (-ks/JPhi), 0, 0, (-kd/JPhi), 0;
     0, -2*l1*(((a*ka*Ue) + b)/JTh), 0, 0, 0, 0];
 
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
Stability = istable(A);
Jord = jordan(A);
% open loop unstable  

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
