function x = minimum_jerk_gen(w,T,dt)

n = length(w)-1;

t = 0:dt:sum(T);

x = zeros(1,length(t));

%get A
F1 = [1 1 1 1 1 1 1 1;
    [0 1 2 3 4 5 6 7]/T(n);
    [0 0 2 6 12 20 30 42]/T(n)^2;
    [0 0 0 6 24 60 120 210]/T(n)^3];

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
B = A^-1*C;


%cal Si
S = [];
S = [S,0];
for k = 1:n
  S = [S,sum(T(1:k))];
end


j = 2;
for k = 1:length(t)
    x(k) = [1 (t(k)-S(j-1))/T(j-1) ((t(k)-S(j-1))/T(j-1))^2  ((t(k)-S(j-1))/T(j-1))^3 ((t(k)-S(j-1))/T(j-1))^4  ((t(k)-S(j-1))/T(j-1))^5  ((t(k)-S(j-1))/T(j-1))^6 ((t(k)-S(j-1))/T(j-1))^7]*B(1+(j-2)*8:8+(j-2)*8);
    if t(k) == S(j)
      j = j+1;
    end
end

end