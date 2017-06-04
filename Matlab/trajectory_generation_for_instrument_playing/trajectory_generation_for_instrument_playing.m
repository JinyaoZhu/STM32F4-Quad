close all;
clear;

H = 0.13;
Y = -0.08-0.14;

do = [-0.21 -0.08-0.14 H];
re = [-0.08 -0.08-0.14 H];
mi = [0.05  -0.08-0.14 H];
fa = [0.17  -0.08-0.14 H];
xi = [-0.21  0.02-0.14 H];
so = [-0.08  0.02-0.14 H];
la = [0.05   0.02-0.14 H];
do_ = [0.17  0.02-0.14 H];

hit = 0;
null = 1;
combo = 2;


waypoints = [0      -0.5       0.4  null;
             0.25    -0.25     0.4  null;
             0.5      0        0.4  null;
             0.25     0.25     0.4  null;
             0       0.5       0.4  null;
             -0.25    0.25     0.4  null;
             -0.5      0       0.4  null;
             -0     -0.5       0.4  null;
    
             %start playing
             do hit;
             do hit;
             so hit;
             so hit;
             la hit;
             la hit;
             so hit;
             fa hit;
             fa hit;
             mi hit;
             mi hit;
             re hit;
             re hit;
             do hit;
             so hit;
             so hit;
             fa hit;
             fa hit;
             mi hit;
             mi hit;
             re hit;
             so hit;
             so hit;
             fa hit;
             fa hit;
             mi hit;
             mi hit;
             re hit;
             do hit;
             do hit;
             so hit;
             so hit;
             la hit;
             la hit;
             so hit;
             fa hit;
             fa hit;
             mi hit;
             mi hit;
             re hit;
             re hit;
             do hit;
             %end playing
             
             %return
             0    Y   H null;
             0     Y  0.3 null;
             0    -0.3  0.4 null;  
             -0.5    -0.5  0.05 null]';

n = length(waypoints);    

%% *************************Get ready**************************************         
M = waypoints(:,1:8);

v = 0.4;

[B1,B2,B3,S] = getPolyCoeff(M,v);


t1 = 0:0.02:S(end);

r1 = [];

for k = 1:length(t1)
    t = t1(k);
    index = find(S >= t,1);
    if index == 1
        r1 = [r1,[M(1,1);M(2,1);M(3,1)]];
    else
        T = S(index)-S(index-1);
        x = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B1(1+(index-2)*6:6+(index-2)*6);
        y = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B2(1+(index-2)*6:6+(index-2)*6);
        z = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B3(1+(index-2)*6:6+(index-2)*6);
        r1 = [r1,[x;y;z]];
    end
end
[~,x] = trajectory_generation_for_two_points(waypoints(1,8),waypoints(1,9),2);
[~,y] = trajectory_generation_for_two_points(waypoints(2,8),waypoints(2,9),2);
[~,z] = trajectory_generation_for_two_points(waypoints(3,8),waypoints(3,9),2);
r1 = [r1,[x';y';z']];

%% **************************Play******************************************
key_time = 1.3;
fly_time = 1.1;
dt = 0.02;

hit_point_base =  length(r1);
hit_point = zeros(1,length(waypoints(1,:))-12);%index where quad should hit

for k = 9:n-4

[~,x1] = trajectory_generation_for_two_points(r1(1,end),waypoints(1,k),fly_time);
[~,y1] = trajectory_generation_for_two_points(r1(2,end),waypoints(2,k),fly_time);
[~,z1] = trajectory_generation_for_two_points(r1(3,end),waypoints(3,k),fly_time);

x2 = waypoints(1,k)*ones(1,floor((key_time-fly_time)/dt));
y2 = waypoints(2,k)*ones(1,floor((key_time-fly_time)/dt));
z2 = waypoints(3,k)*ones(1,floor((key_time-fly_time)/dt));

j = k-8;

if waypoints(4,k) == hit
hit_point(j) = hit_point_base + (j-1)*(key_time/dt)+(fly_time+0.1)/dt;
else
hit_point(j) = -1;
end

x = [x1',x2];
y = [y1',y2];
z = [z1',z2];

r1 = [r1,[x;y;z]]; %#ok<*AGROW>
end

[~,x] = trajectory_generation_for_two_points(waypoints(1,end-4),waypoints(1,end-3),2);
[~,y] = trajectory_generation_for_two_points(waypoints(2,end-4),waypoints(2,end-3),2);
[~,z] = trajectory_generation_for_two_points(waypoints(3,end-4),waypoints(3,end-3),2);
r1 = [r1,[x';y';z']];
%% *************************Return*****************************************
M = waypoints(:,end-3:end);

v = 0.2;

[B1,B2,B3,S] = getPolyCoeff(M,v);


t1 = 0:0.02:S(end);

r3 = zeros(3,length(t1));

for k = 1:length(t1)
    t = t1(k);
    index = find(S >= t,1);
    if index == 1
        r3(1,k) = M(1,1);
        r3(2,k) = M(2,1);
        r3(3,k) = M(3,1);
    else
        T = S(index)-S(index-1);
        r3(1,k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B1(1+(index-2)*6:6+(index-2)*6);
        r3(2,k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B2(1+(index-2)*6:6+(index-2)*6);
        r3(3,k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B3(1+(index-2)*6:6+(index-2)*6);
    end
end

%% ************************************************************************

r = [r1,r3];

t = 0:0.02:length(r)*0.02-0.02;

vr = [zeros(3,1),diff(r,1,2)./0.02];
ar = [zeros(3,1),diff(vr,1,2)./0.02];

figure;
subplot(3,1,1);
plot(t,r(1,:));
grid;
ylabel('X [m]');
subplot(3,1,2);
plot(t,vr(1,:));
grid;
ylabel('vX [m/s]');
subplot(3,1,3);
plot(t,ar(1,:));
grid;
xlabel('t [s]');
ylabel('aX [m/s^2]');

figure;
subplot(3,1,1);
plot(t,r(2,:));
grid;
ylabel('Y [m]');
subplot(3,1,2);
plot(t,vr(2,:));
grid;
ylabel('vY [m/s]');
subplot(3,1,3);
plot(t,ar(2,:));
grid;
xlabel('t [s]');
ylabel('aY [m/s^2]');

figure;
subplot(3,1,1);
plot(t,r(3,:));
grid;
ylabel('Z [m]');
subplot(3,1,2);
plot(t,vr(3,:));
grid;
ylabel('vZ [m/s]');
subplot(3,1,3);
plot(t,ar(3,:));
grid;
xlabel('t [s]');
ylabel('aZ [m/s^2]');

figure;
plot3(r(1,:),r(2,:),r(3,:));
axis equal;
axis([-1 1 -1 1 0 1.5]);
grid;

figure;
plot(r(1,:));
grid;

traj_c_code_gen(r(1,:),r(2,:),r(3,:));

disp(sprintf('max acc: %3.2f, %3.2f, %3.2f',max(abs(ar(1,:))),max(abs(ar(2,:))),max(abs(ar(3,:)))));

str = [];
for k=1:length(hit_point)
 if hit_point(k) ~= -1
  str = [str,sprintf('(i==%d)||',hit_point(k))];
 end
end
disp(strcat('hit point:',str));