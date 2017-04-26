function d_Z = fn_state(~,Z)
global M_c_actual M_d m F_xd F_yd;

I_z=13201;
d_Z=zeros(6,1);

d_Z(1)=Z(3)*cos(Z(5))-Z(4)*sin(Z(5));       %x
d_Z(2)=Z(3)*sin(Z(5))+Z(4)*cos(Z(5));       %y
d_Z(3)=F_xd/m+Z(4)*Z(6);                    %vx
d_Z(4)=F_yd/m-Z(3)*Z(6);                    %vy
d_Z(5)=Z(6);                                %yaw
d_Z(6)=(M_d+M_c_actual)/I_z;                %yaw_dot