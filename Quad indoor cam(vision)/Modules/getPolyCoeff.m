function [B1,B2,B3,T1] = getPolyCoeff(waypoints,v)

d = waypoints(:,2:end) - waypoints(:,1:end-1);
T = sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2)./v;
n = length(T);

B = zeros(8*n,3);

for i = 1:3
    
    w = waypoints(i,:);
    
    %get A
    F1 = [1 1 1 1 1 1 1 1;
        [0 1 2 3 4 5 6 7]/T(n);
        [0 0 2 6 12 20 30 42]/T(n)^2;
        [0 0 0 6 24 60 120 210]/T(n)^3];
    
    % calculation a diagonal Matrix
    A1 = [];
    for k = 1:n-1
        A1 = [A1;zeros(8,8*k-8) order6constraint(T(k),T(k+1)) zeros(8,8*(n-k-1))];
    end
    
    A = [eye(4),zeros(4),zeros(4,8*n-8);
        zeros(4,8*n-8),F1;
        A1];
    
    %get C
    C1 = [];
    for k = 1:n-1
        C1 = [C1;w(k+1);w(k+1);0;0;0;0;0;0];
    end
    
    C = [w(1);0;0;0;w(n+1);0;0;0;C1];
    
    %get the coefficient B
    B(:,i) = A^-1*C;
end
B1 = B(:,1);
B2 = B(:,2);
B3 = B(:,3);
T1 = [0,cumsum(T)];
end