function [t,x] = trajectory_generation_for_two_points(p1,p2,T)

dt = 0.02;

w = [p1,p2];

F1 = [1 1 1 1 1 1;
    [0 1 2 3 4 5]/T;
    [0 0 2 6 12 20]/T^2];

A = [eye(3),zeros(3);
    F1];

C = [w(1);0;0;w(2);0;0];

%get the coefficient B
B = A^-1*C;

S = [0,T];

t1 = 0:dt:S(end);

x = zeros(1,length(t1));

for k = 1:length(t1)
    t = t1(k);
    index = find(S >= t,1);
    if index == 1
        x(k) = w(1,1);
    else
        T = S(index)-S(index-1);
        x(k) = [1 ((t-S(index-1))/T) ((t-S(index-1))/T)^2  ((t-S(index-1))/T)^3 ((t-S(index-1))/T)^4  ((t-S(index-1))/T)^5]*B(1+(index-2)*6:6+(index-2)*6);
    end
end

t = t1';
x = x';

end
