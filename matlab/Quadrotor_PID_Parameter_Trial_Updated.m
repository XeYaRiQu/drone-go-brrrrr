%Position Controls
Kdx = 0.25; % X point Gain
Kdy = 0.25 ;% Y point Gain
Kdz = 0.25; % Z point Gain
m = 0.468; %kg
g = 9.82; %m / s^2 - Gravity

%Angular Accelerations - Body Frame - vdot
I_xx = 4.856 * 10^-3  ;% kg m^2 - Inertia of XX Axis
I_yy = 4.856 * 10^-3 ; %kg m^2 - Inertia of YY axis
I_zz = 8.801 * 10^-3 ;% kg m^2 - Inertia of ZZ Axis

%Angular Velocity Control Parameters
k = 2.980*10^-6  ;% Thrust Coefficient
l = 0.225 ;% m
b = 1.140 * 10^-7 ;% Drag Coefficient

%Declare Initial Conditions Desired Positions
x_desired = -1 ;% X coordinate
y_desired = 2 ; %Y Coordinate
z_desired = 1 ; %Z coordinate
phi_desired = 0.02; % Phi Angle in Radians
theta_desired = 0.02 ; %Theta Angle in Radians
psi_desired = -0.01 ;% Psi Angle in Radians

%Attitude Controller Parameters
Kpphi_attitude= 25 ;
Kptheta_attitude = 2.5; 
Kppsi_attitude = 2.5 ;

%Attitude Rate Controller Parameters - Gains of the PID Controller
Kpphi =2.5;
Kiphi = 0.25; 
Kdphi = 0.25 ;

Kptheta = 2.5;
Kitheta = 0.25; 
Kdtheta = 0.25; 

Kppsi = 2.5;
Kipsi = 0.25; 
Kdpsi = 0.25; 



%Thrust Control Blocks - Gains of the PID Controller
Kzi = 0.25; 
Kzp = 0.25 ;
Kzd = 0.25; 

% Desired Conditions in Parameters
roll_d = 0.01 ;% Desired Roll 
pitch_d = 0.01; %Desired Pitch
yaw_d = 0.01 ;%Desired Yaw
z_d = 1; %Desired Altitude




