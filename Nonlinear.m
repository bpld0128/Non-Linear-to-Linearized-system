%Non-linear
% M = 1000;
% m1 = 100;
% m2 = 100;
% L1 = 20;
% L2 = 10;
% g = 9.8;
% %U = -K*x;
% Xp = zeros(6,1);
% Xp(1)=x(2);
% Xp(2)=( U + (x(4)^2)*m1*L1*sin(x(3)) - (x(6)^2)*m2*L2*sin(x(5)) - g*m1*sin(x(3))*cos(x(3)) - g*m2*sin(x(5))*cos(x(5)) )/(M+m1+m2);
% Xp(3)=x(4);
% Xp(4)=( U*cos(x(3)) - (x(4)^2)*m1*L1*sin(x(3))*cos(x(3)) - (x(6)^2)*m2*L2*sin(x(5))*cos(x(3)) - g*m1*sin(x(3))*(cos(x(3))^2) - g*m2*sin(x(5))*cos(x(5))*cos(x(3)) - g*sin(x(3))*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2) )/(L1*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2 ));
% Xp(5)=x(6); 
% Xp(6)=(U*cos(x(5)) - (x(4)^2)*m1*L1*sin(x(3))*cos(x(5)) - (x(6)^2)*m2*L2*sin(x(5))*cos(x(5)) - g*m1*sin(x(3))*cos(x(3))*cos(x(5)) - g*m2*sin(x(5))*(cos(x(5))^2) - g*sin(x(5))*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2))/(L2*(M + m1*sin(x(3))^2 + m2*sin(x(5))^2 ));

%%%above code is in function nonlinearpendulum.m

%%
%D)Response of a nonlinear system
f1=@nonlinearpendulum;

Tf = 100;
T = [0:0.01:Tf];

%Intial conditions as theta1=0.17 radians, theta2=0.26 radians
x0 = [0; 0; 0.17; 0; 0.26; 0];

%no gains for initial state - dont need these plots
%K = [ 0 0 0 0 0 0];

%Part D gains
%gains from lqr and linearized state
K = [3162.3 6324.5 30486 -4718.8 19580 -24293];


[t,x] = ode45(f1, T, x0, [], K);

figure(1)
title('response of a Nonlinear system to initial conditions')
subplot(3,2,1)
plot(t, x(:,1))
title('x')
xlabel('time (s)')
ylabel('Amplitude (m)')
grid

subplot(3,2,2)
plot(t, x(:,2))
title('xdot')
xlabel('time (s)')
ylabel('Amplitude (m)')
grid

subplot(3,2,3)
plot(t, x(:,3))
title('theta1')
xlabel('time (s)')
ylabel('Amplitude (radians)')
grid

subplot(3,2,4)
plot(t, x(:,4))
title('theta1dot')
xlabel('time (s)')
ylabel('Amplitude (radians)')
grid

subplot(3,2,5)
plot(t, x(:,5))
title('theta2')
xlabel('time (s)')
ylabel('Amplitude (radians)')
grid

subplot(3,2,6)
plot(t, x(:,6))
title('theta2dot')
xlabel('time (s)')
ylabel('Amplitude (radians)')
grid




