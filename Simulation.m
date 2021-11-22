clc
clear
close all

%% System Parameters
y_b=-0;

%%Input
x_in=0.05;
y_in=-0.2;

%%Angle limitations
th1_low=-120*pi/180;
th1_high=-10*pi/180;
th2_low=0;
th2_high=140*pi/180;

%%Masses
m_d=1;              %%Drone
m_b=0.545;          %%Counter balance
m_l1=0.053;         %%Link 1
m_l2=0.035;         %%Link 2
m_p=0.2;            %%Payload

%%Link lengths
l1=0.15;
l2=0.14;

%%COG locations
cog_l1=0.08;
cog_l2=0.082;

%%Starting position
th1s = -pi/2;
th2s = 0;

th1(1)=th1s;
th2(1)=th2s;
n=1;
cond=0;
m=0;
while 1

%% Inverse kinematics
syms t1 t2
eq1=l1*cos(t1)+l2*cos(t1+t2)-x_in;
eq2=l1*sin(t1)+l2*sin(t1+t2)-y_in;
[th1_sol,th2_sol]=solve(eq1,eq2,t1,t2);
[sth1_rows,sth1_columns]=size(th1_sol);
if sth1_rows>1
    p1(1)=real(double(subs(th1_sol(1),t1)));
    p1(2)=real(double(subs(th1_sol(2),t1)));
    p2(1)=real(double(subs(th2_sol(1),t2)));
    p2(2)=real(double(subs(th2_sol(2),t2)));
    if p2(1)>=0
        ph1=p1(1);
        ph2=p2(1);
    else
        ph1=p1(2);
        ph2=p2(2);
    end
else
    ph1=real(double(subs(th1_sol,t1)));
    ph2=real(double(subs(th2_sol,t2)));
end

%% Number of iterations, sets motor speed
res1=1;        %%Resolution of servo movement in degrees
res2=1;

n1=abs(th1(n)-ph1)/(res1*pi/180);
n2=abs(th2(n)-ph2)/(res2*pi/180);
%n1=1;
%n2=1;

%% Infinite while loop
    
    %% Forward kinematics
    th2_abs(n)=th1(n)+th2(n);
    
    xm_l1=cog_l1*cos(th1(n));                           %%Link COM
    xm_l2=l1*cos(th1(n))+cog_l2*cos(th2_abs(n));
    ym_l1=cog_l1*sin(th1(n));
    ym_l2=l1*sin(th1(n))+cog_l2*sin(th2_abs(n));
    
    x_l1(n)=l1*cos(th1(n));                                %%Link length
    x_l2(n)=l1*cos(th1(n))+l2*cos(th2_abs(n));
    y_l1(n)=l1*sin(th1(n));
    y_l2(n)=l1*sin(th1(n))+l2*sin(th2_abs(n));
    
    x_p(n)=l1*cos(th1(n))+l2*cos(th2_abs(n));           %%Payload location
    y_p(n)=l1*sin(th1(n))+l2*sin(th2_abs(n));
    
    
    %% Counter balance location and COG calculation
    
    x_b(n)=-(m_l1*xm_l1+m_l2*xm_l2+m_p*x_p(n))/m_b;     %%Counter balance position
    x_c(n)=x_b(n)*m_b+m_l1*xm_l1+m_l2*xm_l2+m_p*x_p(n); %%COG x
    y_c(n)=y_b*m_b+m_l1*ym_l1+m_l2*ym_l2+m_p*y_p(n);    %%COG Y
    
    
    %% Plot  (display position)
    if n==1
        color='r';
    elseif n==90
        color='g';
    else    
        color='b';
    end
    if rem(n,4)==0 || n==1 || n==90
    plot(x_c(n),y_c(n),join([color,"o"]))
    hold on
    grid on
    axis ([-0.29,0.29,-0.29,0.29])
    speed=1;
    plot(x_b(n),y_b,join([color,"*"]))
    plot(x_p(n),y_p(n),join([color,"*"]))
    plot([0,x_l1(n)],[0,y_l1(n)],color)
    plot([x_l1(n),x_l2(n)],[y_l1(n),y_l2(n)],color)
%     disp(x_b(n))
    end
    
    
    %% Stopping Conditions
    
    % Error (diff between current theta and final)
    e1=th1(n)-ph1;
    e2=th2(n)-ph2;
 
    % Final point reached
    if abs(e1)<(res1*pi/180) && abs(e2)<(res2*pi/180)
        cond=1;
        if cond==1
        fprintf('Point reached.\n');
        break;
        end
    end
    
    % Angle restrictions (hardware limitations)
    if th1(n)<th1_low || th1(n)>th1_high || th2(n)<th2_low || th2(n)>th2_high
        fprintf('Error, point not possible (angle restrictions).\n');
       
    end
    
    % Ensure y<=0
    if y_p>0
        fprintf('Error, point not possible (y cannot be greater than 0).\n')
        break;
    end
    
    % Move servo according to path planning
    if abs(e1)>(res1*pi/180)/2
        if n1<=n2
            th1(n+1)=th1(n)+(n1/n2)*(speed*pi/180)*(ph1-th1(n))/abs(th1(n)-ph1);
        else
            th1(n+1)=th1(n)+(speed*pi/180)*(ph1-th1(n))/abs(th1(n)-ph1);
        end
    else
        th1(n+1)=th1(n);
    end
    if abs(e2)>(res2*pi/180)/2
        if n2<=n1
            th2(n+1)=th2(n)+(n2/n1)*(speed*pi/180)*(ph2-th2(n))/abs(th2(n)-ph2);
        else
            th2(n+1)=th2(n)+(speed*pi/180)*(ph2-th2(n))/abs(th2(n)-ph2);
        end
    else
        th2(n+1)=th2(n);
    end
    n=n+1;
    %pause(0.1)

end
color='r';
plot(x_c(1),y_c(1),join([color,"o"]))
    plot(x_b(1),y_b,join([color,"*"]))
    plot(x_p(1),y_p(1),join([color,"*"]))
    plot([0,x_l1(1)],[0,y_l1(1)],color)
    plot([x_l1(1),x_l2(1)],[y_l1(1),y_l2(1)],color)



xlabel('x(m)');
ylabel('y(m)');
% Point out of reach
if sqrt(x_in^2+y_in^2)>l1+l2+0.01
    fprintf('Error, point not possible (length).\n')
end


%% Everything Below is for Plotting
%% Th1
Figure_th1=figure;
subplot(2,2,1)
hold on
grid on
plot(0:n-1,th1,'b')
title('Theta 1 vs Time')
xlabel('Time Increment (loop)')
ylabel('Theta 1 (rad)')
th1fit=polyfit(0:n-1,th1,ceil(n/20));
tt1=polyval(th1fit,0:n-1);
plot(0:n-1,tt1,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fw1=polyder(th1fit);
w1=round(10000*polyval(fw1,0:n-1))/1000;
hold on
grid on
plot(0:n-1,w1,'b')
title('Omega 1 vs Time')
xlabel('Time Increment (loop)')
ylabel('Omega 1 (rad/incriment)')
axis([0 n -1 1])

subplot(2,2,3)
fa1=polyder(fw1);
a1=round(10000*polyval(fa1,0:n-1))/1000;
hold on
grid on
plot(0:n-1,a1,'b')
title('Alpha 1 vs Time')
xlabel('Time Increment (loop)')
ylabel('Alpha 1 (rad/incriment)')
axis([0 n-1 -1 1])

%% Th2
Figure_th2=figure;
subplot(2,2,1)
hold on
grid on
plot(0:n-1,th2,'b')
title('Theta 2 vs Time')
xlabel('Time Increment (loop)')
ylabel('Theta 2 (rad)')
th2fit=polyfit(0:n-1,th2,ceil(n/20));
tt2=polyval(th2fit,0:n-1);
plot(0:n-1,tt2,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fw2=polyder(th2fit);
w2=round(10000*polyval(fw2,0:n-1))/1000;
hold on
grid on
plot(0:n-1,w2,'b')
title('Omega 2 vs Time')
xlabel('Time Increment (loop)')
ylabel('Omega 2 (rad/incriment)')
axis([0 n -1 1])

subplot(2,2,3)
fa2=polyder(fw2);
a2=polyval(fa2,0:n-1);
hold on
grid on
plot(0:n-1,a2,'b')
title('Alpha 2 vs Time')
xlabel('Time Increment (loop)')
ylabel('Alpha 2 (rad/incriment)')
axis([0 n -1 1])


%% EE X
Figure_xp=figure;
subplot(2,2,1)
hold on
grid on
plot(0:n-1,x_p,'b')
title('EE X Displacement vs Time')
xlabel('Time Increment (loop)')
ylabel('EE X Position (m)')
xfit=polyfit(0:n-1,x_p,ceil(n/20));
xx=polyval(xfit,0:n-1);
plot(0:n-1,xx,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fvx=polyder(xfit);
vx=polyval(fvx,0:n-1);
hold on
grid on
plot(0:n-1,vx,'b')
title('EE X Velocity vs Time')
xlabel('Time Increment (loop)')
ylabel('EE X Velocity (m/incriment)')
axis([0 n -(l1+l2) l1+l2])

subplot(2,2,3)
fax=polyder(fvx);
ax=polyval(fax,0:n-1);
hold on
grid on
plot(0:n-1,ax,'b')
title('EE X Acceleration vs Time')
xlabel('Time Increment (loop)')
ylabel('EE X Acceleration (m/incriment)')
axis([0 n -(l1+l2) l1+l2])


%% EE Y
Figure_yp=figure;
subplot(2,2,1)
hold on
grid on
plot(0:n-1,y_p,'b')
title('EE Y Displacement vs Time')
xlabel('Time Increment (loop)')
ylabel('EE Y Position (m)')
yfit=polyfit(0:n-1,y_p,ceil(n/20));
yy=polyval(yfit,0:n-1);
plot(0:n-1,yy,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fvy=polyder(yfit);
vy=polyval(fvy,1:n);
hold on
grid on
plot(0:n-1,vy,'b')
title('EE Y Velocity vs Time')
xlabel('Time Increment (loop)')
ylabel('EE Y Velocity (m/incriment)')
axis([0 n -(l1+l2) l1+l2])

subplot(2,2,3)
fay=polyder(fvy);
ay=polyval(fay,0:n-1);
hold on
grid on
plot(0:n-1,ay,'b')
title('EE Y Acceleration vs Time')
xlabel('Time Increment (loop)')
ylabel('EE Y Acceleration (m/incriment)')
axis([0 n -(l1+l2) l1+l2])


%% Counter balance displacement
Figure_xb=figure;
subplot(2,2,1)
hold on
grid on
plot(0:n-1,x_b,'b')
title('Counter balance Displacement vs Time')
xlabel('Time Increment (loop)')
ylabel('Counter balance Position (m)')
bfit=polyfit(0:n-1,x_b,ceil(n/20));
bb=polyval(bfit,0:n-1);
plot(0:n-1,bb,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fvb=polyder(bfit);
vb=polyval(fvb,1:n);
hold on
grid on
plot(0:n-1,vb,'b')
title('Counter balance Velocity vs Time')
xlabel('Time Increment (loop)')
ylabel('Counter balance Velocity (m/incriment)')
axis([0 n -0.1 0.1])

subplot(2,2,3)
fab=polyder(fvb);
ab=polyval(fab,0:n-1);
hold on
grid on
plot(0:n-1,ab,'b')
title('Counter balance Acceleration vs Time')
xlabel('Time Increment (loop)')
ylabel('Counter balance Acceleration (m/incriment)')
axis([0 n -0.1 0.1])


%% COG X
Figure_xc=figure;
subplot(2,2,1)
hold on
grid on
axis ([0,n,-0.1,0.1])
plot(0:n-1,x_c,'b')
title('COG X Displacement vs Time')
xlabel('Time Increment (loop)')
ylabel('COG X Position (m)')
xcfit=polyfit(0:n-1,x_c,ceil(n/20));
xxc=polyval(xcfit,0:n-1);
plot(0:n-1,xxc,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fvxc=polyder(xcfit);
vxc=polyval(fvxc,0:n-1);
hold on
grid on
axis ([0,n,-0.1,0.1])
plot(0:n-1,vxc,'b')
title('COG X Velocity vs Time')
xlabel('Time Increment (loop)')
ylabel('COG X Velocity (m/incriment)')
axis([0 n -0.1 0.1])

subplot(2,2,3)
faxc=polyder(fvxc);
axc=polyval(faxc,0:n-1);
hold on
grid on
axis ([0,n,-0.1,0.1])
plot(0:n-1,axc,'b')
title('COG X Acceleration vs Time')
xlabel('Time Increment (loop)')
ylabel('COG X Acceleration (m/incriment)')
axis([0 n -0.1 0.1])


%% COG Y
Figure_yc=figure;
subplot(2,2,1)
hold on
grid on
plot(0:n-1,y_c,'b')
title('COG Y Displacement vs Time')
xlabel('Time Increment (loop)')
ylabel('COG Y Position (m)')
ycfit=polyfit(0:n-1,y_c,ceil(n/20));
yyc=polyval(ycfit,0:n-1);
plot(0:n-1,yyc,'r--')
legend('Actual','Curve-fit')

subplot(2,2,2)
fvyc=polyder(ycfit);
vyc=polyval(fvyc,1:n);
hold on
grid on
plot(0:n-1,vyc,'b')
title('COG Y Velocity vs Time')
xlabel('Time Increment (loop)')
ylabel('COG Y Velocity (m/incriment)')
axis([0 n -0.1 0.1])

subplot(2,2,3)
fayc=polyder(fvyc);
ayc=polyval(fayc,0:n-1);
hold on
grid on
plot(0:n-1,ayc,'b')
title('COG Y Acceleration vs Time')
xlabel('Time Increment (loop)')
ylabel('COG Y Acceleration (m/incriment)')
axis([0 n -0.1 0.1])