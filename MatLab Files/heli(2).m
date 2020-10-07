%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% heli.m
%% Matlab script to be run before Simulink files
%% ACS336 / ACS6336 / ACS6110
%% Last revised: 23.02.2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;      % close all figures
clear all;      % clear workspace variables

%% Define Discrete Time MyDAQ Dynamics
T           = 0.015;            % Sample period (s)
ADC_Vres    = 20/((2^16)-1);    % ADC voltage resolution (V/bit)
Encoder_res = 2*pi/500;         % Encoder resolution (rad/wheel count)
DAC_Vres    = 20/((2^16)-1);    % DAC voltage resolution (V/bit)
DAC_lim_u   = 10;               % DAC upper saturation limit (V)
DAC_lim_l   = 0;                % DAC enforced lower saturation limit (V)
%% Define Continuous Time Helicopter Dynamics
g   = 9.81;     % Gravitational acceleration (ms^-2) 
% Rigid body parameters
% Masses and lengths
m1  = 0.0505;   % mass of fan assembly (kg)
m2  = 0.100;    % mass of counterweight (kg)
l1  = 0.110;    % distance from helicopter arm to elevation axis (m);
l2  = 0.070;    % distance from fan centres to pitch axis (m);
l3  = 0.108;    % distance from counterweight to elevation axis (m);
% Inertias
Je  = 2*m1*(l1^2)+m2*(l3^2);    % Inertia about elevation axis (kg*m^2);
Jt  = Je;                       % Travel axis inertia
Jp  = 2*m1*(l2^2);              % Pitch axis inertia
% Constraints
p_lim_u     = 80*pi/180;    % Upper pitch axis limit (rad)
p_lim_l     = -80*pi/180;   % Lower pitch axis limit (rad)
e_lim_u     = 50*pi/180;    % Upper elevation axis limit (rad)
e_lim_l     = -50*pi/180;   % Lower elevation axis limit (rad)

% %% Ex 1: DETERMINE PITCH AXIS SPRING AND DAMPING COEFFICIENTS %%%%%%%%%%%%%
% % Pitch axis spring and damping constants
% k_s = ;           % Spring constant (kg*m^2*s^-2)
% k_d = ;           % Viscous damping (kg*m^2*s^-1)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Ex 2: DETERMINE POWER AMPLIFIER GAIN AND SATURATION LIMITS %%%%%%%%%%%%%
% Power amplifier
k_a         = ;   % Power amplifier voltage gain
amp_sat_u   = 12;   % Power amplifier upper saturation limit (V)
amp_sat_l   = 0;   % Power amplifier lower saturation limit (V)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Ex 3: CONSTRUCT FAN MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Fan voltage - thrust steady state behaviour
% V_ab   = ;          % Fan voltage input (V)
% Fss_ab = ;          % Steady-state fan thrust output (N)
% % Fan voltage - thrust transient model.
% tau = ;             % 1st order time constant
%
% %% Ex 4: DETERMINE EQUILIBRIUM CONTROL SIGNAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Constant control input required to achieve hover
% U_e = ;           % Voltage output from myDAQ
% 
% %% Ex 5: DEFINE LTI STATE-SPACE CONTROL MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %  Approximate the fan voltage/thrust relationship by an affine           %
% %  function of the form F_ab = alpha*V_ab+beta. Define alpha and beta.    %
% alpha = ;
% beta =  ;
% plot(V_ab,F_ab,'kx');           % plot raw thrust data
% grid on; hold on;
% xlabel('Fan Voltage (V)');
% ylabel('Output Thrust (N)');
% plot(V_ab,alpha*V_ab+beta,'k-'); % plot linear approximation
% %  State vector x:=[elev; pitch; trav; elev_dot; pitch_dot; trav_dot]     %
% %  Note; these states model the dynamics of small perturbations around    %
% %  the state of steady, level hover.                                      %
% %  Define the control model given by x_dot = Ax + Bu, y = Cx + Du         %
% A = ;
% B = ;
% C = ;
% D = ;
% 
% %% Ex 6: Discrete Time Full state feedback control %%%%%%%%%%%%%%%%%%%%%%%%
% % State feedback control design (integral control via state augmentation)
% % x:=[elev; pitch; trav; elev_dot; pitch_dot; trav_dot; int_elev; int_trav]
% % Define augmented system matrices
% Cr      = [1 0 0 0 0 0
%            0 0 1 0 0 0];    % Elevation and travel are controlled outputs
% r       = 2;                                % number of reference inputs
% n       = size(A,2);                        % number of states
% q       = size(Cr,1);                       % number of controlled outputs
% Dr      = zeros(q,2);
% Aaug    = [A zeros(n,r); -Cr zeros(q,r)];
% Baug    = [B; -Dr];
% % Define LQR weighting matrices
% Qx = ;    % State penalty
% Qu = ;    % Control penalty
% % Discrete-Time LQR synthesis
% Kdtaug  = lqrd(Aaug,Baug,Qx,Qu,T);      % DT state-feedback controller
% Kdt     = Kdtaug(:,1:n); Kidt = -Kdtaug(:,n+1:end);
%  Discrete-Time Kalman Filter Design
% sysdt = c2d(ss(A,B,C,D),T,'zoh');     % Generate discrete-time system
% Adt   = sysdt.a; Bdt = sysdt.b; Cdt = sysdt.c; Ddt = sysdt.d;
% %  Kalman filter design; x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v
% Gdt     = 1e-1*eye(n);
% Hdt     = zeros(size(C,1),size(Gdt,2)); % No process noise on measurements
% Rw      = ;   % Process noise covariance matrix
% Rv      = ;   % Measurement noise covariance matrix
% sys4kf  = ss(Adt,[Bdt Gdt],Cdt,[Ddt Hdt],T);
% [kdfilt Ldt] = kalman(sys4kf,Rw,Rv);     % Kalman filter synthesis
% 
%% Output Files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Uncomment these lines when ready to implement your feedback controller  %
% k_amp       = k_a;
% Vfan        = V_ab;
% Thrust      = Fss_ab;
% 
% KF_Initial  = [e_lim_l 0 0 0 0 0];
% Kdi_Initial = [0 0];
% 
% csvwrite('aMatrix.txt',kdfilt.a)
% csvwrite('bMatrix.txt',kdfilt.b)
% csvwrite('cMatrix.txt',kdfilt.c(size(C,1)+1:end,:))
% csvwrite('dMatrix.txt',kdfilt.d(size(C,1)+1:end,:))
% csvwrite('crMatrix.txt',Cr)
% csvwrite('KdMatrix.txt',Kdt)
% csvwrite('KdiMatrix.txt',Kidt)
% csvwrite('FanChar.txt',[Vfan,Thrust])
% csvwrite('ModelParameters.txt',[T,U_e,l1,l2,l3,Jp,Jt,Je,m1,m2,g,...
%     k_amp, amp_sat_u, amp_sat_l, DAC_lim_u, DAC_lim_l,...
%     e_lim_l, e_lim_u, p_lim_l, p_lim_u])
% csvwrite('KF_Initial.txt',KF_Initial)
% csvwrite('Kdi_Initial.txt',Kdi_Initial)