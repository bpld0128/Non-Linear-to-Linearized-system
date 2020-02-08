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

Q = diag([100 1 10000 1 10000 1]);
R = 0.00001;

K = lqr(A,B,Q,R);

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'theta_2_dot'};
inputs = {'r'};
outputs = {'x'; 'theta_1'; 'theta_2'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:30;
r =0.2*ones(size(t));
x0 = [0; 0; 10; 0; 15; 0];
%lsim(sys_cl,r,t,x0);


ob = obsv(sys_ss);
observability = rank(ob);

ObCheck1 = rank(obsv(A,C(1,:))); %x
ObCheck2 = rank(obsv(A,C([2 3],:))); %theta 1, theta 2
ObCheck3 = rank(obsv(A,C([1 3],:))); %x, theta 2
ObCheck4 = rank(obsv(A,C)); %x, theta 1, theta 2

%ObCheck2 with theta1 and theta2 is not observable

%All of the other checks were observable 
% x(t) yes
%(theta1,theta2) no
%(x(t),theta2) yes
%(x(t),theta1,theta2) yes

Vd = 0.001*eye(6); %disturbance covariance
Vn = 0.001*eye(3); %noise covariance

%Building Kalman Filter
[L,P,E] = lqe(A,Vd,C,Vd,Vn)

Vn = 0.001*eye(1);
[L1,P1,E1] = lqe(A,Vd,C(1,:),Vd,Vn)
Vn = 0.001*eye(2);
[L2,P2,E2] = lqe(A,Vd,C([1 3],:),Vd,Vn)
Vn = 0.001*eye(3);
[L3,P3,E3] = lqe(A,Vd,C,Vd,Vn)



%figuring out Bd and Dd



% %xdote = (A-LC)xe +BdUd [simulate this?]
% Ae = A-L*C;
% Be = Vd; %Maybe? Not sure what Bd will be
% Ce = C;
% De = ;
% 
% sys_est = ss(Ae,Be,Ce,De);
% t = 0:0.01:30;
% r =0.2*ones(size(t));
% step(sys_est)


%%%%%%%%%%%%%%%%%%
% Ace = [(A-B*K) (B*K);
%        zeros(size(A)) (A-L*C)];
% Bce = [B;
%        zeros(size(B))];
% Cce = [Cc zeros(size(Cc))];
% Dce = [0;0;0];
% 
% states = {'x' 'x_dot' 'theta_1' 'theta_1_dot' 'theta_2' 'thet_2_dot' 'e1' 'e2' 'e3' 'e4' 'e5' 'e6'};
% inputs = {'F'};
% outputs = {'x'; 'theta_1'; 'theta_2'};
% 
% 
% sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);
% xe0 = [0; 0; 10; 0; 15; 0; 0;0;0;0;0;0];
% 
% 
% t = 0:0.01:30;
% r =0.2*ones(size(t));
% lsim(sys_est_cl,r,t,xe0) 
%%%%%%%%%%%%%%%%%%%%%%


