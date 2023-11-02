% Param
%     --------
%     x : float (Following NED positioning)
%         states, state order:
%         ------------------------
%         1           2           3
%         Position N, Position E, Position D,
%         4           5           6
%         Velocity N, Velocity E, Velocity D,
%         7     8      9
%         Roll, Pitch, Yaw,
%         10 11 12
%         p, q, r
%         -------------------------
%     u : int
%         square of rotation speed of each motor.
%     nu : float - what is this nu for?

%param = struct();
%param.maxthrust = 1000;
%param.kpx = 10;
%param.kix = 15;
%param.kdx = 10; 
%param.mass = 1.4;%kg
%param.gravity = 9.82; %m/s^2
%param.sampletime = 0.005 %seconds
%n = 100;
%Set xyz_desired positions in a 1 row 3 column matrix 
%xyz_desired = zeros([1,3]);
%x = zeros([n, 12]);


%Output parameters must have: s_x, s_y, s_z , thrust , sum_error_x,
%sum_error_y, sum_error_z 
%This will automatically transfer out to Attitude Control MATLAB
function [s_x, s_y, s_z , thrustz, sum_error_x, sum_error_y, sum_error_z] = positioncontrol2(x, xyz_desired,param) % produce out S_x,s_y, sum_error_y,sum_error_x

%Define structure to hold parameters

%Declare parameters = param
maxthrust = param.maxthrust;
%Declare gain parameters of position control at x axis
kpx = param.kpx;
kix = param.kix;
kdx = param.kdx;
%Declare gain parameters of position control y axis
kpy = param.kpy;
kiy = param.kiy; 
kdy = param.kdy;
%Declare gain parameters of position control at z axis
kpz = param.kpz; 
kiz = param.kiz; 
kdz = param.kdz;
%Declare masses of drone and gravity on Earth with sample time needed
mass = param.mass;
gravity = param.gravity;
sampletime = param.sampletime;
%fully declare the desired xyz_desired as the input positions, this will be
%written as vectors together in 1 
x_desired = xyz_desired(1) ;
y_desired = xyz_desired(2);
z_desired = xyz_desired(3);

%% PID Control for X axis with Virtual Control 
% Virtual control is needed because it helps us to manually take care of
% where the positions are?
%Initialise differentiated x_desired = 0
%Velocity of X_desired
x_desired_dot = 0;
%Acceleration of X_desired
x_desired_dotdot = 0;
%Placing into vector on Part 4
x_desired_dot = xyz_desired(4);

%% Virtual Control Command of X axis 
%Check error state of X by desired - current X position
error_x = x_desired - x(1);
%Check error state of X desired dot - Velocity of X_desired - derivative of
%Xd - damoping oscillations and rate of change of the information
error_x_dot = x_desired_dot - x(4);
%Summation of all errors over time later on
sum_error_x = 0;
sum_error_x = sum_error_x + error_x;
%Virtual Control command 
%Following PID Control Equation = Virtual Positioning - this is the error 
%of s-x - Slope Gain term
s_x = kpx*error_x + kdx*error_x_dot + kix*sampletime*sum_error_x;
% Virtual Ux position at Phi Max - Don need for Virtual UX
%virtual_Ux = - real ((kpx*s_x + kdx*error_x_dot + x_desired_dotdot));
%uxMax is a constant where it limits the control so that it would not go
%haywired
%virtual_Ux=min(param.uxMax,max(-param.uxMax,virtual_Ux)); %Phi Max - within the same control limits

%% Y desired Position Coordinates
%% PID Control for Y axis with Virtual Control 
% Virtual control is needed because it helps us to manually take care of
% where the positions are?
%Initialise differentiated y_desired = 0
%Velocity of Y_desired
y_desired_dot = 0;
%Acceleration of Y_desired
y_desired_dotdot = 0;
%Placing into vector on Part 5
y_desired_dot = xyz_desired(5);

%% Virtual Control Command of Y axis 
%Check error state of Y by desired - current Y position
error_y = y_desired - x(2);
%Check error state of Y desired dot - Velocity of Y_desired - derivative of
%Yd - damoping oscillations and rate of change of the information
error_y_dot = y_desired_dot - x(5);
%Summation of all errors over time later on
sum_error_y = 0;
sum_error_y = sum_error_y + error_y;
%Virtual Control command 
%Following PID Control Equation = Virtual Positioning - this is the error 
%of s-x - Slope Gain term
s_y = kpy*error_y + kdy*error_y_dot + kiy*sampletime*sum_error_y;

%% Continue for Z-Axis for S_z at Virtual Control
%% PID Control for Z axis with Virtual Control 
% Virtual control is needed because it helps us to manually take care of
% where the positions are?
%Initialise differentiated Z_desired = 0
%Velocity of Z_desired
z_desired_dot = 0;
%Acceleration of Z_desired
z_desired_dotdot = 0;
%Placing into vector on Part 6
z_desired_dot = xyz_desired(6);

%% Virtual Control Command of Z axis 
%Check error state of Z by desired - current Z position
error_z = z_desired - x(3);
%Check error state of Z desired dot - Velocity of Z_desired - derivative of
%Zd - damoping oscillations and rate of change of the information
error_z_dot = z_desired_dot - x(6);
%Summation of all errors over time later on
sum_error_z = 0;
sum_error_z = sum_error_z + error_z;
%Virtual Control command 
%Following PID Control Equation = Virtual Positioning - this is the error 
%of s-x - Slope Gain term
s_z = kpz*error_z + kdz*error_z_dot + kiz*sampletime*sum_error_z;

%% Thrust Function of the Positional Control  at Z axis - moving up and downwards
% According to Equation 25, cos phi desired and cos theta desired inside
% the equation but not very significant in this - can choose to add in also
% (2 November 2023) remarks - abs function turns -ve values into +ve -
% returns complex magnitude
thrustz = real((gravity + s_z) * (mass));
if abs(thrustz) > maxthrust
    thrustz = maxthrust * sign(thrustz);

end




