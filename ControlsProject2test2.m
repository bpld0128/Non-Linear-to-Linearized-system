Himanshu Singhal
%%
%B) Linearized system around equillibrium points x=0, theta1=0, and theta2=0
syms g M m1 m2 L1 L2
A1 = [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0; 0 0 (-g*(M+m1))/(L1*M) 0 (-g*m2)/(L1*M) 0; 0 0 0 0 0 1; 0 0 (-g*m1)/(L2*M) 0 (-g*(M+m2))/(L2*M) 0];
B1 = [0; 1/M; 0; 1/(L1*M); 0; 1/(L2*M)];
Ranker1 = [B1 A1*B1 (A1^2)*B1 (A1^3)*B1 (A1^4)*B1 (A1^5)*B1];
rank(Ranker1);

%%
%C)Conditions for controllability
syms g M m1 m2 L1

L2=L1;
A1 = [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0; 0 0 (-g*(M+m1))/(L1*M) 0 (-g*m2)/(L1*M) 0; 0 0 0 0 0 1; 0 0 (-g*m1)/(L2*M) 0 (-g*(M+m2))/(L2*M) 0];
B1 = [0; 1/M; 0; 1/(L1*M); 0; 1/(L2*M)];
Ranker1 = [B1 A1*B1 (A1^2)*B1 (A1^3)*B1 (A1^4)*B1 (A1^5)*B1];
rank(Ranker1);

%%
%D)Simulate the response
M = 1000;
m1 = 100;
m2 = 100;
L1 = 20;
L2 = 10;
g = 9.8;

A = [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0; 0 0 (-g*(M+m1))/(L1*M) 0 (-g*m2)/(L1*M) 0; 0 0 0 0 0 1; 0 0 (-g*m1)/(L2*M) 0 (-g*(M+m2))/(L2*M) 0];
B = [0; 1/M; 0; 1/(L1*M); 0; 1/(L2*M)];
C = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
D = [0;0;0];

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'theta_2_dot'};
inputs = {'F'};
outputs = {'x'; 'theta_1'; 'theta_2'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

co = ctrb(sys_ss);
controllability = rank(co);

%%
%D) With given parameters we applied the following LQR controller

%Define LQR parameters
Q = diag([100 1 10000 1 10000 1]);
R = 0.00001;
K = lqr(A,B,Q,R);

%Closed Loop
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'theta_2_dot'};
inputs = {'r'};
outputs = {'x'; 'theta_1'; 'theta_2'};

%Defining the closed loop system
sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

%This code was used for testing the system to find 30 second convergence
%initial(sys_cl,x0)

%Defining the time interval
t = 0:0.01:30;
r =0.2*ones(size(t));

%initial conditions theta1 = 10, theta2 = 15 
x0 = [0; 0; 10; 0; 15; 0]; %radians or degrees? - need to find out

%Linearized system System Response with Initial condition x0 for 30s
%lsim(sys_cl,r,t,x0)
%title('Response of a Linearized system to initial conditions')
%ylabel('Amplitude - x(m) , theta(1,2)(degree)')

%Checking for stability
eig(Ac);

%%
% E)observable
ob = obsv(sys_ss);
observability = rank(ob);

%output vector x(t)
obsv(A,C(1,:));
ObCheck1 = rank(obsv(A,C(1,:))); 

%output vector theta1(t), theta2(t)
obsv(A,C([2 3],:));
ObCheck2 = rank(obsv(A,C([2 3],:))); 

%output vector x(t), theta2(t)
obsv(A,C([1 3],:));
ObCheck3 = rank(obsv(A,C([1 3],:)));

%output vector x(t), theta1(t), theta2(t)
obsv(A,C);
ObCheck4 = rank(obsv(A,C));

%All of the other checks were observable 
% x(t) yes
%(theta1,theta2) no
%(x(t),theta2) yes
%(x(t),theta1,theta2) yes

%%
%F)Luenberger Observer
%Make 10 times faster than slowest controller pole
P = [-1.7 -1.8 -1.9 -2.0 -2.1 -2.2];

%place the poles for first Luenberger with just x(t)
C1 = C(1,:);
L1 = place(A',C1',P)';

Ae1 = A - L1*C1;
Be1 = B;
Ce1 = C1;
De1 = 0;

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'theta_2_dot'};
inputs = {'F'};
outputs = {'x'};

sys_est1 = ss(Ae1,Be1,Ce1,De1,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:10;
r = ones(size(t));
%Theta1 theta2 in degrees
x0 = [0; 0; 10; 0; 15; 0];
%sim for F 1
%lsim(sys_est1,r,t,x0);

%place the poles for first Luenberger with x(t) and theta 2
C2 = C([1 3],:);
L2 = place(A',C2',P)';

Ae2 = A - L2*C2;
Be2 = B;
Ce2 = C2;
De2 = 0;

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'theta_2_dot'};
inputs = {'F'};
outputs = {'x'; 'theta_2'};

sys_est2 = ss(Ae2,Be2,Ce2,De2,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:10;
r = ones(size(t));
x0 = [0; 0; 10; 0; 15; 0];
%sim for F 2
%lsim(sys_est2,r,t,x0);

%place the poles for first Luenberger with x(t) theta 1 and theta 2
C3 = C;
L3 = place(A',C3',P)';

Ae3 = A - L3*C3;
Be3 = B;
Ce3 = C3;
De3 = 0;

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'theta_2_dot'};
inputs = {'F'};
outputs = {'x'; 'theta_1'; 'theta_2'};

sys_est3 = ss(Ae3,Be3,Ce3,De3,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:10;
r = ones(size(t));
x0 = [0; 0; 10; 0; 15; 0];
%sim for F 3
%lsim(sys_est3,r,t,x0);


%%
%G) L3 seems to be the "best" L

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L3*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0;0];

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'thet_2_dot' 'e1' 'e2' 'e3' 'e4' 'e5' 'e6'};
inputs = {'F'};
outputs = {'x'; 'theta_1'; 'theta_2'};


sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);
xe0 = [0; 0; 0.17; 0; 0.26; 0; 0;0;0;0;0;0];


t = 0:0.01:30;
r =zeros(size(t));
%sim for part G
lsim(sys_est_cl,r,t,xe0) 



%%%%%%%%%%%%%%%%%%%%%