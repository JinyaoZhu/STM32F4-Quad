function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor

m = 0.46; % kg
g = 9.81; % m/s/s

I = [0.0005,   0,          2.55e-6;
     0,         0.0005,   0;
     2.55e-6,   0,          0.0007];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = 0.0883/sqrt(2); % m

params.minF = 0.0;
params.maxF = 3.0*m*g;

params.km = 0.5; %factor relate to motor force and it anti-troque

params.color = [0 0 0];

end
