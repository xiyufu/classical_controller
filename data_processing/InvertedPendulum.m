A = [0, 1, 0;68.3929, 0, 0;0, 0, 0];
B = [7.41; 0; 1];
C = eye(3);
D = [0;0;0];

[A1, B1, C1, D1] = c2dm(A,B,C,D,0.01,'zoh');

p = [-3,-3.1,-2.9];
z = exp(0.01*p);
K = acker(A1,B1,z);

pe = 3*p;
ze = exp(0.01*pe);
L = place(A1',C1',ze)';

%%%%%%%%Non-perfect calibration%%%%%%%%
B = [7.41 0; 0, 69.3929;1, 0];
D = [0 0;0 0;0 0];
[A1, B1, C1, D1] = c2dm(A,B,C,D,0.01,'zoh');

K1 = [-K, 0;0, 0, 0, 1];

len = 1000;
x = zeros(3,len+1);

for i = 1:len
    x(:,i+1) = A1*x(:,i) + B1*K1*[x(:,i);0.1];
end

t = [1:len+1]/100;
plot(t,x);grid on;