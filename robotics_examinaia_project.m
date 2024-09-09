%% ***George Charalambous AM:03119706 Robotics I project ***

%% *** Robot (kinematic) model parameters *** 
clear all;
close all;
l(1) = 15.0;  % l0  
l(2) =15.0; % l1 
l(3) =30.0; % l2
l(4) = 30.0; % l3

%% *** sampling period *** 
%% *** for the robot motion, kinematic simulation:
dt = 0.0001; %dt = 0.001; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf=1.0; 	% 1sec duration of motion A-->B 
t=0:dt:Tf; 
t1=0:dt:2*Tf;     % Duration of A-->B and B-->A  = 2sec

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory
%initial =A    final =B
xd0 = 20.00;	  
xd1 = -10.0; 
yd0 = 10.00; 
yd1 = 20.00;  
zd0 =30.00; 
zd1 =30.00; 

% % Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
%% disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...');
disp(' ');
%[xd[k],yd[k],zk[k]] coordinates of the final position over time
%k=0...2sec
xd(1) = xd0; 
yd(1) = yd0; 
zd(1) = zd0;
lambda_x = (xd1-xd0)/Tf; %μέση ταχύτητα μεταβολής του x 
lambda_y = (yd1-yd0)/Tf; %μέση ταχύτητα μεταβολής του y 
lambda_z = (zd1-zd0)/Tf; %μέση ταχύτητα μεταβολής του z 
lambda_xy= (yd1-yd0)/(xd1-xd0);
kmax=Tf/dt ; 
umax=45;

%Calculate xd, yd,zd, for A-->B for t=[0,T/2]
for k=2:3335
   xd(k) = xd(k-1)- 0.5*umax*dt;
   yd(k)= lambda_xy*(xd(k)-xd1)+yd1;
   zd(k) = zd(k-1);
end  
for k=3336:6668
   xd(k) = xd(k-1)-umax*dt;
   yd(k)= lambda_xy*(xd(k)-xd1)+yd1;
   zd(k) = zd(k-1);
end 
for k=6669:10001
   xd(k) = xd(k-1)-0.5*umax*dt;
   yd(k)= lambda_xy*(xd(k)-xd1)+yd1;
   zd(k) = zd(k-1);
end  
%Calculate xd, yd ,zd, for B--> A for t[T/2,T]
for k=10002:13335
   xd(k) = xd(k-1)+0.5*umax*dt;
   yd(k)= lambda_xy*(xd(k)-xd1)+yd1;
   zd(k) = zd(k-1);
end  
for k=13336:16668
   xd(k) = xd(k-1)+umax*dt;
   yd(k)= lambda_xy*(xd(k)-xd1)+yd1;
   zd(k) = zd(k-1);
end 
for k=16669:20001
   xd(k) = xd(k-1)+ 0.5*umax*dt;
   yd(k)= lambda_xy*(xd(k)-xd1)+yd1;
   zd(k) = zd(k-1);
end  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%% compute the reference joint-motion vectors: 
%% {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%% and reference joint (angular) velocities {qd_1(k,i)} 

%% Calculate gd for 3rd joint
%1st solution: elbow down 
gd(:,3)= acos((xd(:).^2 + yd(:).^2 + (zd(:)-l(1)).^2 -l(4)^2- l(3)^2)./ (2*l(3)*l(4)) );
%2nd solution: elbow up
%%gd(:,3)= -acos((xd(:).^2 + yd(:).^2 + (zd(:)-l(1)).^2 -l(4)^2- l(3)^2)./ (2*l(3)*l(4)) ); 
s3= sin(gd(:,3));

%% Calculate gd for 1st joint
% Set an angle "a1" for help
% d11=dcosa1  d12=dsina1   d1=sqrt(d11^2 + d12^2) 
d11= sqrt(xd(:).^2 +(zd(:)-l(1)).^2 - l(2)^2);
%%2nd solution:  d11= -sqrt(xd(:).^2 +(zd(:)-l(1)).^2 - l(2)^2);
d12= l(2);
a1= atan2(d12, d11(:));
gd(:,1)= atan2( zd(:)-l(1) , xd(:) ) - a1(:) ;
s1= sin(gd(:,1));
c1= cos(gd(:,1));

%% Calculate gd for 2nd joint
% Set an angle "a" for help
% d21=dcosa  d22=dsina   d=sqrt(d21^2 + d22^2) 
d21= l(4)*cos(gd(:,3)) + l(3);
d22= l(4)*s3(:);
a2= atan2(d22(:), d21(:));
gd(:,2)= atan2( yd(:) , d11(:) ) - a2; 
c2= cos(gd(:,2));
s2= sin(gd(:,2)); %1st solution: elbow down
%%s2=sin(-gd(:,3))%2nd solution: elbow up

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 
%%(xq1, yq1,zq1) : cartesian position of the 1st link's local reference frame 
xq1 = 0;   
yq1 = 0; 
zq1 = l(1);
%%(xq2, yq2,zq2) : cartesian position of the 2nd link's local reference frame 
xq2 = (-l(2).*s1(:)).';
yq2 = 0;
zq2 = (l(2).*c1(:) + l(1)).';
%%(xq3, yq3,zq3) : cartesian position of the 3nd link's local reference frame 
xq3 = (-l(2).*s1(:)+ l(3).*c2(:).*c1(:) ).';
yq3 = (l(3).*s2(:)).';
zq3 = (l(2).*c1(:) + l(1) +l(3).*s1(:).*c2(:)).';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 


%% Calculate linear speed of final end-points & angular speed for joints
%Initialiasing values
 vx(1) = 0;
 vy(1) = 0;
 vz(1) = 0;
 dq1(1) = 0;
 dq2(1) = 0;
 dq3(1) = 0;

 %linear velocities of the 3rd joints 
for k=2:(2*kmax+1)     
    vx(k) = (xd(k)-xd(k-1))/dt;
    vy(k) = (yd(k)-yd(k-1))/dt;
    vz(k) = (zd(k)-zd(k-1))/dt;
 %angular velocities of the joints q1, q2 ,q3 
    dq1(k) = (gd(k,1)-gd(k-1,1))/dt;
    dq2(k) = (gd(k,2)-gd(k-1,2))/dt;
    dq3(k) = (gd(k,3)-gd(k-1,3))/dt;
end


%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
save;  %% --> save data to 'matlab.mat' file   

%% Positions
fig1 = figure;  
subplot(2,3,1); 
plot(t1,xd); 
ylabel('xd (cm)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t1,yd); 
ylabel('yd (cm)'); 
xlabel('time t (sec)'); 

subplot(2,3,3); 
plot(t1,zd); 
ylabel('zd (cm)'); 
xlabel('time t (sec)');

subplot(2,3,4); 
plot(t1,gd(:,1)); 
ylabel(' Angle of the 1st joint  q1(rad)'); 
xlabel('time t (sec)');  

subplot(2,3,5); 
plot(t1,gd(:,2)); 
ylabel('Angle of the 2nd joint  q2(rad)'); 
xlabel('time t (sec)');    

subplot(2,3,6); 
plot(t1,gd(:,3)); 
ylabel('Angle of the 3rd joint  q3(rad)'); 
xlabel('time t (sec)'); 

%% Velocities linear & angular 
fig2 = figure;  
subplot(2,3,1); 
plot(t1,vx); 
ylabel('Vdx (cm/sec)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t1,vy); 
ylabel('Vdy (cm/sec)'); 
xlabel('time t (sec)'); 

subplot(2,3,3); 
plot(t1,vz); 
ylabel('Vdz (cm/sec)'); 
xlabel('time t (sec)');

subplot(2,3,4); 
plot(t1,dq1); 
ylabel('W1 (rad/sec)'); 
xlabel('time t (sec)');  

subplot(2,3,5); 
plot(t1,dq2); 
ylabel('W2 (rad/sec)'); 
xlabel('time t (sec)');    

subplot(2,3,6); 
plot(t1,dq3); 
ylabel('W3 (rad/sec)'); 
xlabel('time t (sec)');  



%%*** stick diagram --> animate robot motion ... (**optional**) 
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...  

fig4 = figure; 
grid on
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)');
zlabel('z (cm)'); 
plot3(xd,yd,zd,'rs');  %%desired final positions --- A-->B
plot3(0,0,0,'g+');   
dtk=100; %% plot robot position every dtk samples, to animate its motion  
for tk= 1:dtk:kmax+1
    pause(0.5);	%% pause motion to view successive robot configurations    
    plot3([0,xq1],[0,yq1],[0,zq1]);  %1st link
    plot3(xq1,yq1,zq1,'o');          %1st joint 
    plot3([xq1,xq2(tk)],[yq1,yq2],[zq1,zq2(tk)]); %2nd link
    plot3(xq2(tk),yq2,zq2(tk),'o');               %2nd joint
    plot3([xq2(tk),xq3(tk)],[yq2,yq3(tk)],[zq2(tk),zq3(tk)]);%3rd link	
    plot3(xq3(tk),yq3(tk),zq3(tk), 'o'); %3rd joint
    plot3([xq3(tk),xd(tk)],[yq3(tk),yd(tk)],[zq3(tk),zd(tk)]);  	
    plot3(xd(tk),yd(tk),zd(tk),'y+');
end
