clear;
close all;
%%
tic;
waypoints = [
    -0.5 -0.5 0.5 0;
    -0.3  -0.5  0.5 1;
    -0.3 -0.5 0.5   1;
    0  -0.5  0.5 1;
    0  -0.5  0.5 1;
    -0.2  -0.5  0.5 1;
    -0.2  -0.5  0.5 1;
    0.4  -0.5  0.5 1;
    0.4  -0.5  0.5 1;
    ]';
%%

duration = sum(waypoints(4,2:end));

m = length(waypoints(1,:))-1;

S = [0,cumsum(waypoints(4,2:end))];%normalized

dt = 0.02;

t1 = 0:dt:S(end);

% quadprog has three algorithms:
% 
% 'interior-point-convex' (default)
% 'trust-region-reflective'
% 'active-set' (will be removed in a future release)

options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off','MaxIterations',1000);

syms t;
y = object_function(t);
y_diff_4_sq = diff(y,t,4)^2;
tmp={};

for k = 1:m
    
    h=double(fliplr(coeffs(int(y_diff_4_sq,t,S(k),S(k+1)))));
    
    tmp{k} = [0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0;
        0 0 0 0 h(1)*2 h(2) h(3) h(4) h(5);
        0 0 0 0 h(2)  h(6)*2 h(7) h(8) h(9);
        0 0 0 0 h(3)  h(7) h(10)*2 h(11) h(12);
        0 0 0 0 h(4)  h(8) h(11) h(13)*2 h(14);
        0 0 0 0 h(5)  h(9) h(12) h(14) h(15)*2;
        ]; %#ok<*SAGROW>
end

H = blkdiag(tmp{:});

tmp = {};
for k = 2:m+1
    tmp{k} = double(fliplr(coeffs(object_function(S(k)))));
end

Aeq = [1,zeros(1,m*9-1);
    blkdiag(tmp{:});
    zeros(1,1) 1 zeros(1,m*9-2);
    zeros(1,2) 1 zeros(1,m*9-3);
    zeros(1,3) 1 zeros(1,m*9-4);
    zeros(1,4) 1 zeros(1,m*9-5);
    zeros(1,m*9-8) double(fliplr(coeffs(object_function_diff1(S(end)))));
    zeros(1,m*9-7) double(fliplr(coeffs(object_function_diff2(S(end)))));
    zeros(1,m*9-6) double(fliplr(coeffs(object_function_diff3(S(end)))));
    zeros(1,m*9-5) double(fliplr(coeffs(object_function_diff4(S(end)))));
    ];

beq = [waypoints(1,:)';
    zeros(8,1);
    ];

for k=1:m-1
    tmp1 = double(fliplr(coeffs(object_function(S(k+1)))));
    tmp2 = double(fliplr(coeffs(object_function_diff1(S(k+1)))));
    tmp3 = double(fliplr(coeffs(object_function_diff2(S(k+1)))));
    tmp4 = double(fliplr(coeffs(object_function_diff3(S(k+1)))));
    tmp5 = double(fliplr(coeffs(object_function_diff4(S(k+1)))));
    Aeq = [Aeq;
        zeros(1,(k-1)*9) zeros(1,0) tmp1 zeros(1,0) -tmp1 zeros(1,9*(m-k-1));
        zeros(1,(k-1)*9) zeros(1,1) tmp2 zeros(1,1) -tmp2 zeros(1,9*(m-k-1));
        zeros(1,(k-1)*9) zeros(1,2) tmp3 zeros(1,2) -tmp3 zeros(1,9*(m-k-1));
        zeros(1,(k-1)*9) zeros(1,3) tmp4 zeros(1,3) -tmp4 zeros(1,9*(m-k-1));
        zeros(1,(k-1)*9) zeros(1,4) tmp5 zeros(1,4) -tmp5 zeros(1,9*(m-k-1));
        ];
    beq = [beq;zeros(5,1)];
end

A = [];
b = [];

nc = 2;
ub = 0.005;
lb = -0.005;
for k = 1:nc
    t_index = 2;
    t = S(t_index) + (S(t_index+1)-S(t_index))*(k/(nc+1));
    tmp = [zeros(1,9*(t_index-1)) double(fliplr(coeffs(object_function(t)))) zeros(1,9*m-t_index*9)];
    A = [A;-tmp;tmp];
    b = [b;-(lb+waypoints(1,t_index));ub+waypoints(1,t_index)];
end

for k = 1:nc
    t_index = 4;
    t = S(t_index) + (S(t_index+1)-S(t_index))*(k/(nc+1));
    tmp = [zeros(1,9*(t_index-1)) double(fliplr(coeffs(object_function(t)))) zeros(1,9*m-t_index*9)];
    A = [A;-tmp;tmp];
    b = [b;-(lb+waypoints(1,t_index));ub+waypoints(1,t_index)];
end

for k = 1:nc
    t_index = 6;
    t = S(t_index) + (S(t_index+1)-S(t_index))*(k/(nc+1));
    tmp = [zeros(1,9*(t_index-1)) double(fliplr(coeffs(object_function(t)))) zeros(1,9*m-t_index*9)];
    A = [A;-tmp;tmp];
    b = [b;-(lb+waypoints(1,t_index));ub+waypoints(1,t_index)];
end

for k = 1:nc
    t_index = 8;
    t = S(t_index) + (S(t_index+1)-S(t_index))*(k/(nc+1));
    tmp = [zeros(1,9*(t_index-1)) double(fliplr(coeffs(object_function(t)))) zeros(1,9*m-t_index*9)];
    A = [A;-tmp;tmp];
    b = [b;-(lb+waypoints(1,t_index));ub+waypoints(1,t_index)];
end

[cx,fval,eflag] = quadprog(H,[],A,b,Aeq,beq,[],[],[],options);

disp(strcat('exitflag:',num2str(eflag)));
%%
rx = zeros(1,length(t1));
for k = 1:length(t1)
    t = t1(k);
    index = find(S >= t,1);
    if index == 1
        rx(k) = waypoints(1,1);
    else
        rx(k) = [1 t t^2 t^3 t^4 t^5 t^6 t^7 t^8]*cx((index-2)*9+1:(index-2)*9+9);
    end
end

vrx = [0,diff(rx)/dt];

arx = [0,diff(vrx)/dt];
%%
t = t1;

figure(1);
subplot(3,1,1);
plot(t,rx);
grid;
title('Pos X');
subplot(3,1,2);
plot(t,vrx);
grid;
title('Vel X');
subplot(3,1,3);
plot(t,arx);
grid;
title('Acc X');
xlabel('t/s');

ry = zeros(1,length(rx));
rz = 0.3*ones(1,length(rx));

figure;
traj_c_code_gen(rx,ry,rz);
plot3(rx,ry,rz);
grid;
toc
