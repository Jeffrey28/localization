%% 응용로봇공학 2017 프로젝트 1
% GPS와 IMU를 이용한 Localization
% 작성자: 홍효성
clear; close all; clc;
addpath('./sub_functions');

%% 시뮬레이션 옵션 설정
animation_enable = false;   % true인 경우 시뮬레이션 종료 후 차량 및 위성을 animation으로 plot (속도 매우 느림 주의!)
step_time = 0.01;
sim_time_end = 500;
T=step_time;    % step time

%% Kalman filter parameter 설정 (GPS 위치 데이터 필터링에 사용)
Ak1 = eye(3);
Hk1 = eye(3);
Pk1 = eye(3);
Qk1 = eye(3);
Rk1 = eye(3)*10000;

Ak2 = [1, 0, T, 0;
    0, 1, 0, T;
    0, 0, 1, 0;
    0, 0, 0, 1];
Bk2 = [T^2/2, 0;
    0, T^2/2;
    T, 0;
    0, T];
Hk2 = [1, 0, 0, 0;
    0, 1, 0, 0];
Pk2 = eye(4);
Qk2 = eye(4);
Rk2 = eye(2)*5000;

%% 위성 객체 생성
satellite1 = Vehicles(17000000, pi/6, pi/6);
satellite2 = Vehicles(13000000, pi/2, pi/2);
satellite3 = Vehicles(10000000, pi/10, pi/4);
satellite4 = Vehicles(7000000, pi/20, pi/3);

%% Vehicle path tracking control
global M_c M_c_actual M_d m F_xd F_yd;
% vehicle parameter
m=6762;             % mass(kg)
L=1;             % vehicle length (m)
t_w=1.948;         % distance between right and left wheels (m) = tread
I_z=13201;         % moment of inertia of z-axis (kgm^2)
L_f=1.8788; L_m=0.2784; L_r=1.3216;    % perpendicular length from C.G. to each axle
tire_radius=0.547;            % tire radius (m)

% [X,Y,vx,vy,yaw,yaw_d]'=[Z1,Z2,Z3,Z4,Z5,Z6]'   state
%Z(1)=0; Z(2)=0;         % vehicle position (X, Y) from global coordinate
%Z(3)=0; Z(4)=0;         % vehicle velocity (vx, vy) from vehicle coordinate
%Z(5)=0; Z(6)=0;         % vehicle heading angle (rad, rad/s) from global coordinate
Z=[34.69,136.41,0,0,0,0];
Z2=[34.69,136.41,0,0];

% control gains
K_vp=10;
K_vi=0;
K_yd=10;
K_yp=20;

% observer gains
P=10;
l=P*I_z;
eta=25*I_z;

% observer initial value
r_hat=0;
M_d_hat=0;
M_d=0;
M_c=0;

% initial conditions
% T=step_time;    % step time
e_v=0;      % velocity error
e_v_sum=0;  % velocity error integration
e_y=0;      % yaw error
e_yd=0;     % yaw rate error
L_pv=0;     % preview distance (m)
e_l=0;      % lateral position error (m)

F_y=zeros(1,6);                                             % lateral tire force (front, middle, rear)
C_a=17453;                                                  % cornering stiffness (front, middle, rear)
alpha=zeros(1,6);                                           % side slip angle
W1=1; W2=1; W3=1; W4=1; W5=1; W6=1;                         % weighting coefficient of each wheel
F_z1=m/6*9.81; F_z2=m/6*9.81; F_z3=m/6*9.81;
F_z4=m/6*9.81; F_z5=m/6*9.81; F_z6=m/6*9.81;                % estimated vertical tire force
acc_y=0;                                                    % vehicle total lateral acceleration
F_x_des=zeros(6,1);

% force limit parameter
torque_limit=3416;  % Nm
force_limit=torque_limit/tire_radius;     % gear ratio= 1:1

% waypoints assignment
WP=load('Path.txt');       % waypoint [x;y;velocity]
i=2;
[WP_size,~]=size(WP);
final_time=sim_time_end;
u_wp=0;
yaw_d=0;

%% state 초기 값 입력
position_save = zeros(sim_time_end/step_time, 15);   % [time, actual, GPS acquired, GPS estimated, GPS+IMU estimated, Stereo vision estimated]
satellite1_save = zeros(sim_time_end/step_time, 4);
satellite2_save = zeros(sim_time_end/step_time, 4);
satellite3_save = zeros(sim_time_end/step_time, 4);
satellite4_save = zeros(sim_time_end/step_time, 4);
X_hat1 = [WP(1,1); WP(1,2); 0];   % GPS kalman filter state의 초기값
X_hat2 = [WP(1,1); WP(1,2); 0; 0];   % GPS + IMU kalman filter state의 초기값
acquired_position = [0;0;0];
iter=0;

%% plot test
for t=0:T:final_time
    iter = iter + 1;
    %% car move
    WP1=WP(i-1,:);              % waypoint 새로 할당
    WP2=WP(i,:);
    dn=sqrt((WP2(1)-WP1(1))^2+(WP2(2)-WP1(2))^2);       % waypoint 거리
    yaw_wp=atan2((WP2(2)-WP1(2)),(WP2(1)-WP1(1)));      % waypoint direction (rad)
    u_wp=((Z(1)-WP1(1))*(WP2(1)-WP1(1))+(Z(2)-WP1(2))*(WP2(2)-WP1(2)))/dn^2;
    while u_wp>1           % u_wp>1 : 차량이 주어진 두 waypoint 범위를 벗어남 -> 새로운 waypoint 할당
        if i==(WP_size)   % 주어진 경로가 더이상 없으면 종료
            break;
        end
        i=i+1;
        processInPercent=i/WP_size*100;
        fprintf('Progress: %.2f%% , Velocity : %1.1fm/s\n',processInPercent, Z(3));
        WP1=WP(i-1,:);              % waypoint 새로 할당
        WP2=WP(i,:);
        dn=sqrt((WP2(1)-WP1(1))^2+(WP2(2)-WP1(2))^2);
        yaw_wp=atan2((WP2(2)-WP1(2)),(WP2(1)-WP1(1)));      % waypoint direction (rad)
        u_wp=((Z(1)-WP1(1))*(WP2(1)-WP1(1))+(Z(2)-WP1(2))*(WP2(2)-WP1(2)))/dn^2;
    end
    if i==(WP_size)   % 주어진 경로가 더이상 없으면 종료
        break;
    end
    x_d = WP1(1)+u_wp*(WP2(1)-WP1(1));    % desired x position (perpendicular point from the path)
    y_d = WP1(2)+u_wp*(WP2(2)-WP1(2));    % desired y position (perpendicular point from the path)
    if u_wp > 0
        v_d = WP1(3)+u_wp*(WP2(3)-WP1(3));    % desired velocity
    else
        v_d = WP1(3);   % 차량 초기 위치가 waypoint 보다 한참 뒤에 있을 경우 u_wp < 0가 되어 마이너스 속도(v_d < 0) 발생하는 것 방지
    end

    v_d = v_d*exp(-0.5*abs(e_l));   % 횡 방향 위치 오차가 커지면 속도 명령을 줄임 (2016.11.10)
    
    % preview distance
    if abs(Z(3))<L
        L_pv=L;
    else
        L_pv=abs(Z(3))*L;               % preview distance update
    end
    i_pv=i;
    d_rest=sqrt((WP(i_pv,1)-x_d)^2+(WP(i_pv,2)-y_d)^2);
    if L_pv > d_rest
        L_pv_rest=L_pv-d_rest;
        while(L_pv_rest>0)
            if i_pv==(WP_size)
                break;
            end
            i_pv=i_pv+1;
            d_rest=sqrt((WP(i_pv,1)-WP(i_pv-1,1))^2+(WP(i_pv,2)-WP(i_pv-1,2))^2);
            L_pv_rest=L_pv_rest-d_rest;
        end
        L_pv_rest=d_rest+L_pv_rest;
        yaw_wp_pv=atan2((WP(i_pv,2)-WP(i_pv-1,2)),(WP(i_pv,1)-WP(i_pv-1,1)));
        x_pv=WP(i_pv-1,1)+L_pv_rest*cos(yaw_wp_pv);
        y_pv=WP(i_pv-1,2)+L_pv_rest*sin(yaw_wp_pv);
        if i_pv==(WP_size)
            x_pv=WP(i_pv,1);
            y_pv=WP(i_pv,2);
        end
    else
        x_pv=x_d+L_pv*cos(yaw_wp);
        y_pv=y_d+L_pv*sin(yaw_wp);
    end
    
    % lateral position error
    e_l=sqrt((x_d-Z(1))^2+(y_d-Z(2))^2)*sign(y_d-Z(2));
    if abs(e_l)>20
        break;      % 경로에서 너무 많이 벗어날 경우 시뮬레이션 중지
    end
    yaw_d_before = yaw_d;
    yaw_d=atan2((y_pv-Z(2)),(x_pv-Z(1)));    % desired heading angle
    
    % error
    yaw=Z(5);
    if yaw>=0
        if yaw_d<(yaw-pi)
            e_y=yaw_d+2*pi-yaw;
        else
            e_y=yaw_d-yaw;
        end
    else
        if yaw_d>(yaw+pi)
            e_y=(yaw_d-2*pi)-yaw;
        else
            e_y=yaw_d-yaw;
        end
    end
    
    if yaw_d_before>=0
        if yaw_d<(yaw_d_before-pi)
            e_ydes=yaw_d+2*pi-yaw_d_before;
        else
            e_ydes=yaw_d-yaw_d_before;
        end
    else
        if yaw_d>(yaw_d_before+pi)
            e_ydes=(yaw_d-2*pi)-yaw_d_before;
        else
            e_ydes=yaw_d-yaw_d_before;
        end
    end
    
    yaw_d_dot = e_ydes/T;
    e_yd = -Z(6);
    e_v=v_d-Z(3);
    e_v_sum=e_v_sum+e_v*T;
    
    % tire slip angle (front, middle, rear)
    if Z(4)==0
        alpha=zeros(1,6);
    else
        % sign convention
        alpha(1)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_f*cos(Z(5))-Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_f*sin(Z(5))-Z(6)*t_w/2*cos(Z(5))));
        alpha(3)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_m*cos(Z(5))-Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_m*sin(Z(5))-Z(6)*t_w/2*cos(Z(5))));
        alpha(5)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))-Z(6)*L_r*cos(Z(5))-Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))+Z(6)*L_r*sin(Z(5))-Z(6)*t_w/2*cos(Z(5))));
        
        alpha(2)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_f*cos(Z(5))+Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_f*sin(Z(5))+Z(6)*t_w/2*cos(Z(5))));
        alpha(4)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_m*cos(Z(5))+Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_m*sin(Z(5))+Z(6)*t_w/2*cos(Z(5))));
        alpha(6)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))-Z(6)*L_r*cos(Z(5))+Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))+Z(6)*L_r*sin(Z(5))+Z(6)*t_w/2*cos(Z(5))));
    end
    
    % slip angle sign convention
    for i3=1:6
        if alpha(i3) > pi
            alpha(i3) = alpha(i3) - 2*pi;
        elseif alpha(i3) < -pi
            alpha(i3) = alpha(i3) + 2*pi;
        end
    end    
    
    % lateral tire force with respect to its slip angle
    for i3=1:6
        if abs(alpha(i3)>0.09)  % if slip angle > 0.09 rad, then slip angle = 0.09 fix
            F_y(i3)=-C_a*0.09*sign(alpha(i3));
        else
            F_y(i3)=-C_a*alpha(i3);
        end
    end
    
    % vehicle total lateral force
    F_yd=F_y(1)+F_y(2)+F_y(3)+F_y(4)+F_y(5)+F_y(6);    
    
    % vehicle moment (M_d, M_c)
    M_d=L_f*(F_y(1)+F_y(2))+L_m*(F_y(3)+F_y(4))-L_r*(F_y(5)+F_y(6));
    M_c=I_z*(K_yd*e_yd+K_yp*e_y); 
    
    % vehicle desired longitudinal force
    F_xd=m*(K_vp*(e_v)+K_vi*e_v_sum);       % desired vehicle force
    
    % vehicle longitudinal force at each wheel
    A=[2*(W1/(F_z1^2 + 1)+W5/(F_z5^2 + 1)),0,2*W5/(F_z5^2 + 1),0;
        0,2*(W2/(F_z2^2 + 1)+W6/(F_z6^2 + 1)),0,2*W6/(F_z6^2 + 1);
        2*W5/(F_z5^2 + 1),0,2*(W3/(F_z3^2 + 1)+W5/(F_z5^2 + 1)),0;
        0,2*W6/(F_z6^2 + 1),0,2*(W4/(F_z4^2 + 1)+W6/(F_z6^2 + 1))];
    B=[W5/(F_z5^2 + 1)*F_xd-2*W5/(F_z5^2 + 1)*M_c/t_w;
        W6/(F_z6^2 + 1)*F_xd+2*W6/(F_z6^2 + 1)*M_c/t_w;
        W5/(F_z5^2 + 1)*F_xd-2*W5/(F_z5^2 + 1)*M_c/t_w;
        W6/(F_z6^2 + 1)*F_xd+2*W6/(F_z6^2 + 1)*M_c/t_w];
    F_x1234=A\B;               % inv(A)*B
    F_x_des(1)=F_x1234(1);                % longitudinal tire force at each wheel
    F_x_des(2)=F_x1234(2);
    F_x_des(3)=F_x1234(3);
    F_x_des(4)=F_x1234(4);
    F_x_des(5)=F_xd/2-M_c/t_w-F_x_des(1)-F_x_des(3);
    F_x_des(6)=F_xd/2+M_c/t_w-F_x_des(2)-F_x_des(4);
    
    % longitudinal tire force limit
    if abs(F_x_des(1))>force_limit || abs(F_x_des(2))>force_limit || abs(F_x_des(3))>force_limit || abs(F_x_des(4))>force_limit || abs(F_x_des(5))>force_limit || abs(F_x_des(6))>force_limit
        limit_ratio=force_limit/max(abs(F_x_des));
        for j=1:6
            F_x_des(j)=F_x_des(j)*limit_ratio;
        end
    end
    
    M_c_actual=t_w/2*(F_x_des(2)+F_x_des(4)+F_x_des(6)-F_x_des(1)-F_x_des(3)-F_x_des(5));  % actual M_c
    % disturbance moment observer
    r_hat=r_hat+T/I_z*M_d_hat+T/I_z*M_c+T*P*(Z(6)-r_hat);   % yaw rate estimate
    M_d_hat=M_d_hat+T*eta*(Z(6)-r_hat);                     % disturbance moment estimate    
    
    % ODE45
    [t2,Z_ode]=ode45(@fn_state,[0 T],Z);
    Z=Z_ode(size(t2,1),:);
    actual_position = Z(1:2);   
    
    %% satellite move for a step time
    satellite1.move();
    satellite2.move();
    satellite3.move();
    satellite4.move();
    
    %% GPS position acqusition
    acquired_position_old = acquired_position;
    acquired_position = solvePosition(acquired_position,satellite1.getPosition(),satellite2.getPosition(),satellite3.getPosition(),satellite4.getPosition(),...
        satellite1.getDistance([Z(1:2),0]),satellite2.getDistance([Z(1:2),0]),satellite3.getDistance([Z(1:2),0]),satellite4.getDistance([Z(1:2),0]));
    
    %% kalman filter state update (GPS)
    Zk1 = acquired_position;   % measurement
    
    Kk1 = Pk1*Hk1'/(Hk1*Pk1*Hk1'+Rk1);
    X_hat_old1 = X_hat1;
    X_hat1 = X_hat1+Kk1*(Zk1-Hk1*X_hat1);
    gps_position = X_hat1;
    Pk1 = Ak1*(eye(3)-Kk1*Hk1)*Pk1*Ak1'+Qk1;
    X_hat1 = Ak1*X_hat1;
    
    %% IMU 계산
      
    % 측정된 gps position으로부터 heading 계산
    gps_heading = atan2(acquired_position(2)-acquired_position_old(2), acquired_position(1)-acquired_position_old(1));
%     gps_heading = Z(5);
    
    % IMU acceleration
    d_z = fn_state(0,Z);
    imu_acc_local = d_z(3:4); % local acceleration
    imu_acc_global = [cos(gps_heading), -sin(gps_heading); sin(gps_heading), cos(gps_heading)]*imu_acc_local;
    imu_acc_global = imu_acc_global + randn*0.1;
    
    % kalman filter state update (IMU)
    Zk2 = acquired_position(1:2);   % measurement
    
    Kk2 = Pk2*Hk2'/(Hk2*Pk2*Hk2'+Rk2);
%     Kk2 = zeros(4,2);
    X_hat2 = X_hat2+Kk2*(Zk2-Hk2*X_hat2);
    gps_imu_position = X_hat2;
    Pk2 = Ak2*(eye(4)-Kk2*Hk2)*Pk2*Ak2'+Qk2;
    X_hat2 = Ak2*X_hat2+Bk2*imu_acc_global;
    
    %% Stereo Vision을 통한 차량 위치 계산
    [fxL, fxR] = get_SV_focus([Z(1); Z(2); 0]);
    SV_Position = get_SV_position(fxL, fxR);
    
    %% position 저장
    position_save(iter,:) = [t, actual_position, 0, acquired_position', gps_position', gps_imu_position(1:2)', SV_Position'];
    satellite1_save(iter,:) = [t, satellite1.global_position'];
    satellite2_save(iter,:) = [t, satellite2.global_position'];
    satellite3_save(iter,:) = [t, satellite3.global_position'];
    satellite4_save(iter,:) = [t, satellite4.global_position'];
end
position_save(iter:end, :)=[];
satellite1_save(iter:end, :)=[];
satellite2_save(iter:end, :)=[];
satellite3_save(iter:end, :)=[];
satellite4_save(iter:end, :)=[];

%% GPS localization result plot
% figure(1)
% plot3(position_save(:,5),position_save(:,6),position_save(:,7),'.', 'Color', [0.8 0.8 0.8])
% hold on
% plot3(position_save(:,8),position_save(:,9),position_save(:,10),'b','LineWidth',3)
% hold on
% plot3(WP(:,1),WP(:,2),zeros(size(WP,1),1),'r--','LineWidth',3)
% grid on
% axis equal
% legend('GPS raw data', 'estimated', 'Path')
% xlabel('X [m]')
% ylabel('Y [m]')
% zlabel('Z [m]')

figure(2)
plot(position_save(:,5),position_save(:,6), 'Color', [0.8 0.8 0.8], 'LineWidth',1)
hold on
plot(position_save(:,8),position_save(:,9), 'b', 'LineWidth',1)
plot(position_save(:,11),position_save(:,12), 'm', 'LineWidth',1)
plot(position_save(:,13),position_save(:,14), 'g', 'LineWidth',1)
plot(position_save(:,2),position_save(:,3), 'c', 'LineWidth',1)
plot(WP(:,1),WP(:,2),'r--','LineWidth',3)
grid on
axis equal
legend('GPS raw data', 'GPS KF estimate', 'GPS + IMU KF estimate', 'Stereo vision', 'Driven path', 'Desired path')
xlabel('X [m]')
ylabel('Y [m]')

%% 3D animation
if animation_enable == true
    satellite1_global_orbit = satellite1.global_orbit;
    satellite2_global_orbit = satellite2.global_orbit;
    satellite3_global_orbit = satellite3.global_orbit;
    satellite4_global_orbit = satellite4.global_orbit;
    
    figure(3)
    for i=1:100:iter-1
%         subplot(2,1,1)
        % satellite orbit plot
        plot3(WP(:,1),WP(:,2),zeros(size(WP,1)),'r--','LineWidth',3)
        hold on
        plot3(satellite1_global_orbit(1,:),satellite1_global_orbit(2,:),satellite1_global_orbit(3,:),'r--')
        hold on
        plot3(satellite2_global_orbit(1,:),satellite2_global_orbit(2,:),satellite2_global_orbit(3,:),'g--')
        hold on
        plot3(satellite3_global_orbit(1,:),satellite3_global_orbit(2,:),satellite3_global_orbit(3,:),'b--')
        hold on
        plot3(satellite4_global_orbit(1,:),satellite4_global_orbit(2,:),satellite4_global_orbit(3,:),'m--')
        hold on
        
        plot3(position_save(i,2),position_save(i,3),position_save(i,4), 'ks')
        hold on
        plot3(satellite1_save(i,2), satellite1_save(i,3), satellite1_save(i,4), 'r*')
        hold on
        plot3(satellite2_save(i,2), satellite2_save(i,3), satellite2_save(i,4), 'g*')
        hold on
        plot3(satellite3_save(i,2), satellite3_save(i,3), satellite3_save(i,4), 'b*')
        hold on
        plot3(satellite4_save(i,2), satellite4_save(i,3), satellite4_save(i,4), 'm*')
        hold off
        axis equal
        grid on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        sim_status = sprintf('Simulation time: %3.2f / %3.2f s',position_save(i,1), position_save(1,end));
        title(sim_status,'FontSize',15)
        
%         subplot(2,1,2)
%         plot(WP(:,1),WP(:,2),'r--','LineWidth',3)
%         hold on
%         plot(position_save(i,2),position_save(i,3), 'ks')
%         hold off
%         axis equal
%         grid on
%         xlabel('X')
%         ylabel('Y')
%         zlabel('Z')
%         sim_status = sprintf('Simulation time: %3.2f / %3.2f s',position_save(i,1), position_save(1,end));
%         title(sim_status,'FontSize',15)
        
        pause(step_time/10)
    end
end