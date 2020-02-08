function Xp = nonlinearpendulum(theta, x, K)

M = 1000;
m1 = 100;
m2 = 100;
L1 = 20;
L2 = 10;
g = 9.8;

U = -K*x;

Xp = zeros(6,1);

Xp(1)=x(2);
Xp(2)=( U - (x(4)^2)*m1*L1*sin(x(3)) - (x(6)^2)*m2*L2*sin(x(5)) - g*m1*sin(x(3))*cos(x(3)) - g*m2*sin(x(5))*cos(x(5)) )/(M + m1*sin(x(3))^2 + m2*sin(x(5))^2 );
Xp(3)=x(4);
Xp(4)=( U*cos(x(3)) - (x(4)^2)*m1*L1*sin(x(3))*cos(x(3)) - (x(6)^2)*m2*L2*sin(x(5))*cos(x(3)) - g*m1*sin(x(3))*(cos(x(3))^2) - g*m2*sin(x(5))*cos(x(5))*cos(x(3)) - g*sin(x(3))*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2) )/(L1*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2 ));
Xp(5)=x(6); 
Xp(6)=(U*cos(x(5)) - (x(4)^2)*m1*L1*sin(x(3))*cos(x(5)) - (x(6)^2)*m2*L2*sin(x(5))*cos(x(5)) - g*m1*sin(x(3))*cos(x(3))*cos(x(5)) - g*m2*sin(x(5))*(cos(x(5))^2) - g*sin(x(5))*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2))/(L2*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2 ));




