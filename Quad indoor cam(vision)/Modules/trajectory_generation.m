clear;
close all;

dt = 0.02;

% waypoints = [
%              0 0 0.2;
%              0 0 0.5;
%              0 -0.1 0.5;
%              0 -0.2 0.5;
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
%              0 -0.2 0.5;
%              0 -0.1 0.5;
%              0 -0.05 0.5;
%              0 0 0.5;
%              0 0 0.2;
%                        ]';

% waypoints = [
%              -0.5 -0.2 0.3;
%              0   0.3 0.3;
%              0.5 -0.2 0.3;]';

% waypoints = [
%              -0.5 0 0.3;
%              -0.25 0.3 0.3;
%              0    0 0.3;
%              0.25 -0.3 0.3;
%              0.5 0 0.3]';

% waypoints = [
%              0 0 0.2;
%              0 0 0.5;
%              0 -0.1 0.5;
%              0 -0.2 0.5;
% 
%              -0.565 -0.2 0.5;
%              -0.565 -0.2 0.3;
%              -0.565 -0.2 0.2;
%              -0.565 -0.2 0.3;
%              -0.565 -0.2 0.5;
%              -0.37 -0.2 0.5;
%              -0.37 -0.2 0.3;
%              -0.37 -0.2 0.2;
%              -0.37 -0.2 0.3;
%              -0.37 -0.2 0.5;
%              -0.19 -0.2 0.5;
%              -0.19 -0.2 0.3;
%              -0.19 -0.2 0.2;
%              -0.19 -0.2 0.3;
%              -0.19 -0.2 0.5;
%              0.007 -0.2 0.5;
%              0.007 -0.2 0.3;
%              0.007 -0.2 0.2;
%               0.007 -0.2 0.3;
%              0 -0.2 0.5;
%              0 -0.1 0.5;
%              0 0 0.5;
%              0 0 0.2;
%                        ]';

waypoints = [
               0    0  0.2;
               0    -0.5  0.6;
                       ]';

% waypoints = [
%              0  0 0.8 0.3;
%              0  0 0.6 0.3;
%              0  0 0.8 0.3;
%              0  0 0.6 0.3;
%              0  0 0.8 0.3;
%              -0.05  0 0.8 0.3;
%              -0.1  0 0.8 0.3;
%              -0.1  0 0.7 0.3;
%              -0.1  0 0.6 0.3;
%              -0.1  0 0.7 0.3;
%              -0.1  0 0.8 0.3;
%                        ]';

% x = [];
% y = [];
% N = 20;
% for k = 1:N
%     x = [x,0.4*cos(8*2*pi*k/N)]; 
%     y = [y,0.4*sin(8*2*pi*k/N)-0.2]; 
% end
% waypoints = [x;y;0.3*ones(1,N)];
                   
%% 
                   
M = waypoints(1:3,:);
%v = waypoints(4,2:end);
v = 0.8;
[B1,B2,B3,S] = getPolyCoeff(M,v);

%% 

t1 = 0:dt:S(end);

x = zeros(1,length(t1));
y = zeros(1,length(t1));
z = zeros(1,length(t1));

for k = 1:length(t1)
    t = t1(k);
    index = find(S >= t,1);
    if index == 1
        x(k) = M(1,1);
        y(k) = M(2,1);
        z(k) = M(3,1);
    else
        T = S(index)-S(index-1);
        x(k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5  ((t-S(index-1))/T)^6 ((t-S(index-1))/T)^7]*B1(1+(index-2)*8:8+(index-2)*8);
        y(k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5  ((t-S(index-1))/T)^6 ((t-S(index-1))/T)^7]*B2(1+(index-2)*8:8+(index-2)*8);
        z(k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5  ((t-S(index-1))/T)^6 ((t-S(index-1))/T)^7]*B3(1+(index-2)*8:8+(index-2)*8);
    end
end

% t1 = 0:0.02:10;
% x = zeros(1,length(t1));
% y = zeros(1,length(t1));
% z = zeros(1,length(t1));
% for k = 1:length(t1)
%         x(k) = 0.4*cos(2*2*pi*t1(k)/t1(end));
%         y(k) = 0.4*sin(2*2*pi*t1(k)/t1(end));
%         z(k) = 0.3;
% end

%% 


t = t1;

traj_c_code_gen(x,y,z);

figure(1);
plot3(x,y,z);
grid;
axis equal;
%axis([-2,2,-2,2,0,2]);
xlabel('x');
ylabel('y');
zlabel('z');

figure(2);
subplot(3,1,1);
plot(t,x);
title('X');
xlabel('t/s');
ylabel('position/m');
grid on;
subplot(3,1,2);
vx = diff(x)/dt;
plot(t(1:length(vx)),vx);
xlabel('t/s');
ylabel('velocity/m/s');
grid on;
subplot(3,1,3);
ax = diff(vx)/dt;
plot(t(1:length(ax)),ax);
xlabel('t/s');
ylabel('acceleration/m/s^2');
grid on;

figure(3);
subplot(3,1,1);
plot(t,y);
title('Y');
xlabel('t/s');
ylabel('position/m');
grid on;
subplot(3,1,2);
vy = diff(y)/dt;
plot(t(1:length(vy)),vy);
xlabel('t/s');
ylabel('velocity/m/s');
grid on;
subplot(3,1,3);
ay = diff(vy)/dt;
plot(t(1:length(ay)),ay);
xlabel('t/s');
ylabel('acceleration/m/s^2');
grid on;

figure(4);
subplot(3,1,1);
plot(t,z);
title('Z');
xlabel('t/s');
ylabel('position/m');
grid on;
subplot(3,1,2);
vz = diff(z)/dt;
plot(t(1:length(vz)),vz);
xlabel('t/s');
ylabel('velocity/m/s');
grid on;
subplot(3,1,3);
az = diff(vz)/dt;
plot(t(1:length(az)),az);
xlabel('t/s');
ylabel('acceleration/m/s^2');
grid on;

disp('max acc');
[max(abs(ax)),max(abs(ay)),max(abs(az))] %#ok<*NOPTS>
% figure(5);
% comet3(x,y,z);