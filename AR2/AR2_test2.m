clc
clear
close

% startup_rvc
Qtheta = [0,-pi/2,-pi/4,0,pi/3,0];%betaΪ0��ʱ��ͻ�������� �����������㣿

L0=Link([0     169.77      0   0   0],'modified'); %�������˵�D-H���� d0 = 169.77
L1=Link([0     0           0    0  0],'modified'); %�������˵�D-H����
L2=Link([0     0           64.2   -pi/2  0],'modified'); %�������˵�D-H���� a1 = 64.2
L3=Link([0     0           305     0],'modified'); %�������˵�D-H���� a2 = 305
L4=Link([0    222.63       0   -pi/2  0],'modified'); %�������˵�D-H���� a3 = 0
L5=Link([0     0           0   pi/2  0],'modified'); %�������˵�D-H����
L6=Link([0     36.5          0   -pi/2  0],'modified'); %�������˵�D-H���� d6 = 36.5 a6 = 0
robot = SerialLink([L0 L1 L2,L3,L4,L5,L6],'name','AR2-modified');

Qtheta1 = [0,Qtheta];
robot.plot(Qtheta1);
c=robot.fkine(Qtheta1)

%����
dataXYZ = AR2_kinematicsEquation(Qtheta)
%����
[dataTheta,flag] = AR2_inverseKinematics(dataXYZ);

resTheta = [];
%�򵥷�Χɸѡ��ѡ������������Ľ�
for i=1:8
    if(flag(i) == 0)
      resTheta = [resTheta;dataTheta(i,:)];
    end
end

resTheta

% dataTheta1 = [dataTheta,flag']

% 
% robot.ikine(c)

