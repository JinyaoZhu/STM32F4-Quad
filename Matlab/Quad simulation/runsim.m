close all;
clear;

%% Trajectory generation with waypoints
% x = [];
% y = [];
%
% for k = 1:40
%     x = [x,0.5*cos(16*2*pi*k/40)];
%     y = [y,0.5*sin(16*2*pi*k/40)];
% end
%
% waypoints = [x;y;0.3*ones(1,40)];

% waypoints = [
%              0 0 0.4;
%              0.2 0 0.5;
%              0  0 0.5;
%              0.8 0 0.5;
%              1 0 0.4]';
waypoints = [
             -0.75   -0.5   0.2;
             -0.5   0      0.5;
             -0.3   0      0.5;
             -0.1   0      0.5;
             0.0   0      0.5;
             0.1   0      0.5;
             0.3   0      0.5;
             0.5   0      0.5;
             0.75    0.5    0.2;]';

% waypoints = [
%              0 0 0.2;
%              0 0 0.5;
%              0 -0.1 0.5;
%              0 -0.2 0.5;
%              -0.3 -0.2 0.5;
%              -0.565 -0.2 0.5;
%              -0.565 -0.2 0.3;
%              -0.565 -0.2 0.15;
%              -0.565 -0.2 0.3;
%              -0.565 -0.2 0.5;
%              -0.37 -0.2 0.5;
%              -0.37 -0.2 0.3;
%              -0.37 -0.2 0.15;
%              -0.37 -0.2 0.3;
%              -0.37 -0.2 0.5;
%              -0.19 -0.2 0.5;
%              -0.19 -0.2 0.3;
%              -0.19 -0.2 0.15;
%              -0.19 -0.2 0.3;
%              -0.19 -0.2 0.5;
%              0.007 -0.2 0.5;
%              0.007 -0.2 0.3;
%              0.007 -0.2 0.15;
%               0.007 -0.2 0.4;
%              0 -0.2 0.5;
%              0 -0.1 0.5;
%              0 0 0.5;
%              0 0 0.2;
%                        ]';



% waypoints = [-0.5 0 0.3;...
%             -0.35 0.3 0.3;...
%             0.0 0.0 0.3;...
%             0.35 -0.3 0.3;...
%             0.5 0 0.3]';
%% ************************************************************************      
controlhandle = @controller;
trajhandle = @traj_generator;
trajhandle([],[],waypoints);

[t, state] = simulation_3d(trajhandle, controlhandle);
