%AR2 ����

function [outputTheta,theta_flag] = AR2_inverseKinematics(intputXYZ)

%���� (������Z-Y-X)
% alpha = EulerZ; 
% beta = EulerY;
% gamma = EulerX;

alpha = deg2rad(intputXYZ(6)); %�Ƕ�ת��Ϊ����
beta = deg2rad(intputXYZ(5));
gamma = deg2rad(intputXYZ(4));

ca=cos(alpha);
sa=sin(alpha);
cb=cos(beta);
sb=sin(beta);
cy=cos(gamma);
sy=sin(gamma);

R_input = [
    ca*cb,ca*sb*sy-sa*cy,ca*sb*cy+sa*sy,intputXYZ(1);
    sa*cb,sa*sb*sy+ca*cy,sa*sb*cy-ca*sy,intputXYZ(2);
    -sb,cb*sy,cb*cy,intputXYZ(3);
    0,0,0,1
];

%DHģ����صĲ���
d0 = 169.77;
d4 = 222.63;
d6 = 36.5;

a1 = 64.2;
a2 = 305;
a3 = 0;
a6 = 0;

%%%%%%% �������
%   T0_s=[
%       1,0,0,0;
%       0,1,0,0;
%       0,0,1,-d0;
%       0,0,0,1
%      ];
%  
%   Tt_6 = [
%       1,0,0,a6;
%       0,1,0,0;
%       0,0,1,-d6;
%       0,0,0,1 
%   ];
% 
%  T_input1 = T0_s * R_input * Tt_6;
%%%%%% ����������

%�ĳɲ�Ҫ����
% [ Nx, Ox, Ax,      Px - Ax*d6 + Nx*a6]
% [ Ny, Oy, Ay,      Py - Ay*d6 + Ny*a6]
% [ Nz, Oz, Az, Pz - d0 - Az*d6 + Nz*a6]
% [  0,  0,  0,                       1]
T_input1 = R_input;
T_input1(1,4) = R_input(1,4) - R_input(1,3)*d6 + R_input(1,1)*a6;
T_input1(2,4) = R_input(2,4) - R_input(2,3)*d6 + R_input(2,1)*a6;
T_input1(3,4) = R_input(3,4) - d0 - R_input(3,3)*d6 + R_input(3,1)*a6;

 nx=T_input1(1,1);
 ox=T_input1(1,2);
 ax=T_input1(1,3);
 px=T_input1(1,4);
 
 ny=T_input1(2,1);
 oy=T_input1(2,2);
 ay=T_input1(2,3);
 py=T_input1(2,4);
 
 nz=T_input1(3,1);
 oz=T_input1(3,2);
 az=T_input1(3,3);
 pz=T_input1(3,4);

 theta_flag = zeros(1,8);%�ж��Ƿ��н� 0��ʾ�н�
 
LimitRange=[
    -165,165;
    -110,110;
    -90,70;
    -160,160;
    -120,120;
    -400,400;
    ];

LimitRange = deg2rad(LimitRange);
LimitOmega=[250,250,250,420,590,600];%һ������Ϊ����ٶȣ���ʾ�˶�����Ȩ������

theta = zeros(8,6);

% һ����8����
 for i=1:8
     if(i<5)
         theta(i,1) = atan2(py,px) - atan2(0,sqrt(px*px+py*py));
     else
         theta(i,1)= atan2(py,px) - atan2(0,-sqrt(px*px+py*py));
     end
     
     h=px*px+py*py+pz*pz+a1*a1;
     g=2*a1*cos(theta(i,1))*px+2*a1*sin(theta(i,1))*py+a3*a3+d4*d4+a2*a2;
     k=(h-g)/(2*a2);
     
     theta3_temp = a3*a3+d4*d4-k*k; %��Ҫ���������뱣֤������
     if(theta3_temp < 0)
        theta(i,:) = zeros(1,6);%������������ǰ�������ʼֵ
        theta_flag(i) = 1;
        continue;
     else
         if(i<3||i>6) 
            theta(i,3)=atan2(a3,d4)-atan2(k,sqrt(theta3_temp));
         else
             theta(i,3)=atan2(a3,d4)-atan2(k,-sqrt(theta3_temp));
         end
     end
             
%      if(i<3||i>6) 
%          theta(i,3)=atan2(a3,d4)-atan2(k,sqrt(a3*a3+d4*d4-k*k));
%      else
%          theta3_temp = a3*a3+d4*d4-k*k; %��Ҫ���������뱣֤������
%          if(theta3_temp < 0)
%              theta(i,:) = [-1,-1,-1,-1,-1,-1];
%              continue;
%          else
%              theta(i,3)=atan2(a3,d4)-atan2(k,-sqrt(theta3_temp));
%          end
% %          theta(i,3)=atan2(a3,d4)-atan2(k,-sqrt(a3*a3+d4*d4-k*k));
%          
%      end
     
     s23_temp1 = cos(theta(i,1))*px+sin(theta(i,1))*py-a1;
     s23_temp2 = pz*pz+s23_temp1*s23_temp1;
     s23=((-a3-a2*cos(theta(i,3)))*pz+(cos(theta(i,1))*px+sin(theta(i,1))*py-a1)*(a2*sin(theta(i,3))-d4))/s23_temp2;
     c23 = ((-d4+a2*sin(theta(i,3)))*pz+(cos(theta(i,1))*px+sin(theta(i,1))*py-a1)*(a2*cos(theta(i,3))+a3))/s23_temp2;
     
     theta(i,2) = atan2(s23,c23)-theta(i,3);
     theta(i,4)  =atan2(-ax*sin(theta(i,1))+ay*cos(theta(i,1)),-ax*cos(theta(i,1))*cos(theta(i,2)+theta(i,3))-ay*sin(theta(i,1))*cos(theta(i,2)+theta(i,3))+az*sin(theta(i,2)+theta(i,3)));
     
     if(mod(i,2)==0) %ע�������C���Ե�����
         theta(i,4) = theta(i,4) + pi;
     end
     
    s5 = - ax*(cos(theta(i,1))*cos(theta(i,2)+theta(i,3))*cos(theta(i,4))+sin(theta(i,1))*sin(theta(i,4)))- ay*(sin(theta(i,1))*cos(theta(i,2)+theta(i,3))*cos(theta(i,4))-cos(theta(i,1))*sin(theta(i,4)))+ az*sin(theta(i,2)+theta(i,3))*cos(theta(i,4));     
    c5= - (ax*cos(theta(i,1))*sin(theta(i,2)+theta(i,3))+ ay*sin(theta(i,1))*sin(theta(i,2)+theta(i,3))+ az*cos(theta(i,2)+theta(i,3)));
 
    theta(i,5)=atan2(s5,c5);
    
    s6=-nx*(cos(theta(i,1))*cos(theta(i,2)+theta(i,3))*sin(theta(i,4))-sin(theta(i,1))*cos(theta(i,4)))-ny*(sin(theta(i,1))*cos(theta(i,2)+theta(i,3))*sin(theta(i,4))+cos(theta(i,1))*cos(theta(i,4)))+nz*sin(theta(i,2)+theta(i,3))*sin(theta(i,4));
    c6=-ox*(cos(theta(i,1))*cos(theta(i,2)+theta(i,3))*sin(theta(i,4))-sin(theta(i,1))*cos(theta(i,4)))-oy*(sin(theta(i,1))*cos(theta(i,2)+theta(i,3))*sin(theta(i,4))+cos(theta(i,1))*cos(theta(i,4)))+oz*sin(theta(i,2)+theta(i,3))*sin(theta(i,4));
    
    theta(i,6) = atan2(s6,c6);
%     theta_flag(i) = 1;%��ʾ�н�
    
%     theta(i,:),i
     
    inRange = 1;
    for j=1:6 %һ��6��theta
        if(theta(i,j) >= LimitRange(j,1) && theta(i,j)<= LimitRange(j,2))
            continue;
        else
            inRange = 0;
            break;
        end
    end
    
    if(inRange == 0)
        theta_flag(i) = 2;
        continue;
    end
   
 end
 
 outputTheta= rad2deg(theta);
%  flag = theta_flag