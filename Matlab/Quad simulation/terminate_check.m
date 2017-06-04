function [ terminate_cond ] = terminate_check( x, time, stop, pos_tol, vel_tol, time_tol)
%TERMINATE_CHECK Check termination criteria, including position, velocity and time


% Check position and velocity and still time for each quad
pos_check = (norm(x(1:3) - stop) < pos_tol);
vel_check = (norm(x(4:6)) < vel_tol);

% Check total simulation time
time_check = time > time_tol;

if (pos_check && vel_check && (time>1))
    terminate_cond = 1; % Robot reaches goal and stops, successful
elseif time_check
    terminate_cond = 2; % Robot doesn't reach goal within given time, not complete
else
    terminate_cond = 0;
end

end
