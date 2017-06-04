function sdot = quadEOM(t, s, controlhandle, trajhandle,params)
% Wrapper function for solving quadrotor equation of motion

%
% INPUTS:
% t             - 1 x 1, time
% s             - 15 x 1, state vector = [x, y, z, xd, yd, zd, yaw, pitch, roll, p, q, r, mx, my, mz]
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from sys_params() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 15 x 1, derivative of state vector s
%

% convert state to quad stuct for control
current_state = s;

% Get desired_state
desired_state = trajhandle(t, current_state);

% get control outputs
[F, M] = controlhandle(t, current_state,desired_state, params);

% compute derivative
% Limit the force and moments due to actuator limits
L = params.arm_length;

%relation between [force moments] and propellers' force 
T = 0.25*[1 -1/L  -1/L -1/(sqrt(2)*L*params.km);
          1 -1/L   1/L  1/(sqrt(2)*L*params.km);
          1  1/L   1/L -1/(sqrt(2)*L*params.km);
          1  1/L  -1/L  1/(sqrt(2)*L*params.km)];
      
inv_T = T^-1;

F_prop = T*[F;M];
F_prop = max(min(F_prop, params.maxF/4), params.minF/4);%clampped

F = inv_T(1,:)*F_prop;
M = inv_T(2:4,:)*F_prop;

% Assign states
r = s(1:3);
r_dot = s(4:6);
yaw = s(7);
pitch = s(8);
roll = s(9);
omega = s(10:12);
I = params.I;


%rotation matrix body-to-world
wRb = YPRToDCM(s(7),s(8),s(9));

% Acceleration
accel = 1/params.mass*wRb * [0; 0; F] - [0; 0; params.gravity];

% Angular velocity
euler_dot = [1 tan(pitch)*sin(roll) tan(pitch)*cos(roll);
             0      cos(roll)       -sin(roll);
             0 sin(roll)*sec(pitch) cos(roll)*sec(pitch)]*omega;

% Angular acceleration
omega_dot = I^(-1)*(M - cross(omega,I*omega));

Tm = 0.07; %time constant of motor

% Assemble sdot
sdot = zeros(12,1);
sdot(1)  = r_dot(1);
sdot(2)  = r_dot(2);
sdot(3)  = r_dot(3);
sdot(4)  = accel(1);
sdot(5)  = accel(2);
sdot(6)  = accel(3);
sdot(7)  = euler_dot(3);
sdot(8)  = euler_dot(2);
sdot(9)  = euler_dot(1);
sdot(10) = omega_dot(1);
sdot(11) = omega_dot(2);
sdot(12) = omega_dot(3);
% sdot(13) = (-s(13) + M(1))/Tm;
% sdot(14) = (-s(14) + M(2))/Tm;
% sdot(15) = (-s(15) + M(3))/Tm;
sdot(13:15)=zeros(3,1);
end
