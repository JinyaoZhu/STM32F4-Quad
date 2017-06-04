function [F, M] = controller(t, state,des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

% persistent last_t last_omega_error
% 
% if t == 0
%     last_t =0;
%     last_omega_error = zeros(3,1);
% end

pos = state(1:3);
vel = state(4:6);
yaw = state(7);
pitch = state(8);
roll = state(9);
omega = state(10:12);

% if t <= 1
%     des_state.pos(3) = 0;
% elseif t <= 2
%     des_state.pos(3) = 0.25;
% else
%     des_state.pos(3) = 0.25;
% end

error_pos = des_state.pos-pos;
error_vel = des_state.vel-vel;

% if norm(des_state.vel) > 0.01
%  unit_vel = des_state.vel/norm(des_state.vel);
%  error_pos_n = error_pos - unit_vel'*error_pos;
% else
%  error_pos_n = error_pos;
% end



%desire acc in referrent frame
des_acc_x = des_state.acc(1) + 9.0*(error_vel(1))+25*(error_pos(1));
des_acc_y = des_state.acc(2) + 9.0*(error_vel(2))+25*(error_pos(2));
des_acc_z = des_state.acc(3) + 9.0*(error_vel(3))+25*(error_pos(3)) + params.gravity;

% Thurst
F = params.mass*des_acc_z/(cos(pitch)*cos(roll));

pitch_comm =  asin((1/params.gravity)*( des_acc_x*cos(yaw) + des_acc_y*sin(yaw)));
roll_comm   = -asin((1/(params.gravity*cos(roll)))*(-des_acc_x*sin(yaw) + des_acc_y*cos(yaw)));
yaw_comm = des_state.yaw;

if pitch_comm > 50*pi/180
    pitch_comm = 50*pi/180;
elseif pitch_comm < -50*pi/180;
    pitch_comm = -50*pi/180;
end

if roll_comm > 50*pi/180
    roll_comm = 50*pi/180;
elseif roll_comm < -50*pi/180;
    roll_comm = -50*pi/180;
end

% if t <= 1
%     yaw_comm = 0;
% elseif t <= 2
%     yaw_comm = 0.25;
% else
%     yaw_comm = 0;
% end

omega_des = zeros(3,1);

omega_des(1) = 30*(roll_comm - roll);
omega_des(2) = 30*(pitch_comm - pitch);
omega_des(3) = 20*(yaw_comm - yaw);


omega_error = omega_des-omega;

% dt = t-last_t;
% 
% if dt ~= 0
% d_omega_error = (omega_error-last_omega_error)/dt;
% else
% d_omega_error = zeros(3,1);
% end
% 
% last_omega_error = omega_error;
% last_t = t;

d_omega_error = zeros(3,1);

% Moment
M(1,1) = params.I(1,1)*(70*omega_error(1) + 5*d_omega_error(1));
M(2,1) = params.I(2,2)*(70*omega_error(2) + 5*d_omega_error(2));
M(3,1) = params.I(3,3)*(70*omega_error(3) + 5*d_omega_error(3));
%M = M + cross(state.omega,params.I*state.omega);
% =================== Your code ends here ===================

end
