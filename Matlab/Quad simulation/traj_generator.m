function [ desired_state ] = traj_generator(t, state, waypoints)

persistent B1 B2 B3 S waypoints0
if nargin > 2
    v = 0.5;
    [B1,B2,B3,S] = getPolyCoeff(waypoints,v);%get the coefficient matrix
    waypoints0 = waypoints;
else
    if t == inf
        desired_state.pos = waypoints0(:,end);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
    else
        index = find(S >= t,1); 
        
        if index == 1 % t == 0
            x = waypoints0(1,1);
            y = waypoints0(2,1);
            z = waypoints0(3,1);
            vx = 0;
            vy = 0;
            vz = 0;
            ax = 0;
            ay = 0;
            az = 0;
        else
            T = S(index)-S(index-1);
            if isempty(T) % all trajectories are generated
                x = waypoints0(1,end);
                y = waypoints0(2,end);
                z = waypoints0(3,end);
                vx = 0;
                vy = 0;
                vz = 0;
                ax = 0;
                ay = 0;
                az = 0;
            else % generate trajectories
                p0 = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5  ((t-S(index-1))/T)^6 ((t-S(index-1))/T)^7];
                v0 = [0 1/T 2*(t-S(index-1))/T^2 3*(t-S(index-1))^2/T^3 4*(t-S(index-1))^3/T^4 5*(t-S(index-1))^4/T^5  6*(t-S(index-1))^5/T^6 7*(t-S(index-1))^6/T^7];
                a0 = [0 0   2/T^2 6*(t-S(index-1))/T^3 12*(t-S(index-1))^2/T^4  20*(t-S(index-1))^3/T^5  30*(t-S(index-1))^4/T^6  42*(t-S(index-1))^5/T^7];
                
                x = p0*B1(1+(index-2)*8:8+(index-2)*8);
                y = p0*B2(1+(index-2)*8:8+(index-2)*8);
                z = p0*B3(1+(index-2)*8:8+(index-2)*8);
                
                vx = v0*B1(1+(index-2)*8:8+(index-2)*8);
                vy = v0*B2(1+(index-2)*8:8+(index-2)*8);
                vz = v0*B3(1+(index-2)*8:8+(index-2)*8);
                
                ax = a0*B1(1+(index-2)*8:8+(index-2)*8);
                ay = a0*B2(1+(index-2)*8:8+(index-2)*8);
                az = a0*B3(1+(index-2)*8:8+(index-2)*8);
            end
        end
        desired_state.pos = [x;y;z];
        desired_state.vel = [vx;vy;vz];
        desired_state.acc = [ax;ay;az];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
end
%         desired_state.pos = zeros(3,1);
%         desired_state.vel = zeros(3,1);
%         desired_state.acc = zeros(3,1);
%         desired_state.yaw = 0;
end

