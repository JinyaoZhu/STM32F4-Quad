function [t_out, s_out] = simulation_3d(trajhandle, controlhandle)
% ***************** QUADROTOR SIMULATION *****************


% real-time
real_time = true;

% max time
max_time = 50;

% parameters for simulation
params = sys_params;

%% **************************** FIGURES *****************************
disp('Initializing figures...');
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
set(gcf,'Renderer','OpenGL');

%% *********************** INITIAL CONDITIONS ***********************
disp('Setting initial conditions...');
tstep    = 0.05; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = ceil(max_time/cstep); % max iteration
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;

x       = [des_start.pos(1);des_start.pos(2);des_start.pos(3);zeros(3,1);0;0;0;zeros(6,1)];% innit state

pos_tol = 0.01;
vel_tol = 0.01;

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');
% Main loop
for iter = 1:max_iter
    
    tic;

    timeint = time:tstep:time+cstep;

    % Initialize quad plot
    if iter == 1
        QP = QuadPlot(1, x, params, max_iter, h_3d);
        desired_state = trajhandle(time, x);
        QP.UpdateQuadPlot(x,[desired_state.pos;desired_state.vel], time);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end

    % Run simulation
    [~, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle,trajhandle,params), timeint, x);
    x    = xsave(end, :)';
    
    time = time + cstep; % Update simulation time

    % Update quad plot
    desired_state = trajhandle(time, x);
    QP.UpdateQuadPlot(x,[desired_state.pos;desired_state.vel], time);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time))

    %saveas(h_3d,sprintf('capture%d.jpg',iter));
    
    t = toc;
    
    % Check to make sure ode45 is not timing out
    if(t> cstep*100)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    %Check termination criteria
    if terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate saved variables
QP.TruncateHist();

%Plot position
h_pos = figure('Name','Quad position');
plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
% Plot velocity
h_vel = figure('Name','Quad velocity');
plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');
% Plot euler
h_euler = figure('Name','Quad euler');
plot_state(h_euler, QP.state_hist(7:9,:), QP.time_hist, 'euler', 'vic');

if(~isempty(err))
    error(err);
end

% figure;
% a = [zeros(1,1/0.02),0.25*ones(1,1/0.02),0.25*ones(1,1/0.02),0.25*ones(1,1/0.02+1)];
% plot(QP.time_hist,QP.state_hist(7,:),QP.time_hist,a,'--','LineWidth',2);
% title('att control');
% legend('output','desired');
% xlabel('t(s)');ylabel('z(m)');
% grid;

% figure;
% error_x = QP.state_des_hist(1,:) - QP.state_hist(1,:);
% error_y = QP.state_des_hist(2,:) - QP.state_hist(2,:);
% error_z = QP.state_des_hist(3,:) - QP.state_hist(3,:);
% error_sum_x  = sum(error_x.^2);
% error_sum_y  = sum(error_y.^2);
% error_sum_z  = sum(error_z.^2);
% plot(QP.time_hist,error_x,QP.time_hist,error_y,QP.time_hist,error_z,'LineWidth',1);
% title('trajectory tracking');
% legend('error x','error y','error z');
% xlabel('t(s)');ylabel('error(m)');
% grid;

disp('Finished.')

sum_error = sum((QP.state_hist(3,:)-QP.state_des_hist(3,:)).^2)

t_out = QP.time_hist;
s_out = QP.state_hist;

end
