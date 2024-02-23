%Position Controls
Kdx = 1.3; % X point Gain
Kpxy = 3.5;% % Kp gain for both x and y
Kdy = 1.3;% Y point Gain
Kdz = 0.30; % Z point Gain
m = 0.468; %kg
g = 9.82; %m / s^2 - Gravity
%altitude_i = 50; %metres
roll_i = 0; %radians
yaw_i = 0; %radians
pitch_i = 0; %radians

%Angular Accelerations - Body Frame - vdot
I_xx = 4.856 * 10^-3  ;% kg m^2 - Inertia of XX Axis - might need to change this due to the weight and inertia
I_yy = 4.856 * 10^-3 ; %kg m^2 - Inertia of YY axis
I_zz = 8.801 * 10^-3 ;% kg m^2 - Inertia of ZZ Axis

%Angular Velocity Control Parameters
k = 2.980*10^-6  ;% Thrust Coefficient
l = 0.225 ;% m
b = 1.140 * 10^-7 ;% Drag Coefficient

%Declare Initial Conditions Desired Positions
x_desired = 15;% X coordinate
y_desired = 10; %Y Coordinate
z_desired = 0.1;
z_initial= 0.0; %Setting initial coordinate
%z_desired = 0;% ; %Z coordinate
%phi_desired = 0.02; % Phi Angle in Radians
%theta_desired = 0.02 ; %Theta Angle in Radians
%psi_desired = -0.01 ;% Psi Angle in Radians

%Attitude Controller Parameters
%Kpphi_attitude= 2.5;
%Kptheta_attitude = 2.5; 
%Kppsi_attitude = 2.5 ;

%Attitude Rate Controller Parameters - Gains of the PID Controller - This
%could be different from the actual intial attitude controller
%Kpphi =25;
%Kiphi = 2.5; 
%Kdphi = 0 ;

%Kptheta = 2.5;
%Kitheta = 0.25; 
%Kdtheta = 0; 

%Kppsi = 2.5;
%Kipsi = 0.5; 
%Kdpsi = 0; 

%Thrust Control Blocks - Gains of the PID Controller
%Kzi = 0.25; 
%Kzp = 0.25 ;
%Kzd = 0; 

% Desired Conditions in Parameters
roll_d = 0.10; % Desired Roll  - this replaces Phi Desired
pitch_d = 0.09; %Desired Pitch - this replaces Theta Desired
yaw_d = 0.05 ;%Desired Yaw - this replaces Psi Desired in the parameter
altitude_d = 5; %Desired Altitude
z_dot_desired = 0.01; %Desired Linear Velocity

%Gain PD Controllers
Kpz = 100;
Kdz = 1;




