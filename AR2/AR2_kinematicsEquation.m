%AR2 正解

function outputXYZ = AR2_kinematicsEquation(intputTheta)

Q1 = intputTheta(1);
Q2 = intputTheta(2);
Q3 = intputTheta(3);
Q4 = intputTheta(4);
Q5 = intputTheta(5);
Q6 = intputTheta(6);

d0 = 169.77;
d4 = 222.63;
d6 = 36.5;

a1 = 64.2;
a2 = 305;
a3 = 0;
a6 = 0;

T01 = [cos(Q1),-sin(Q1),0,0;
    sin(Q1), cos(Q1),0,0;
    0, 0, 1, 0;
    0, 0, 0, 1];

T12 = [cos(Q2),-sin(Q2),0,a1;
    0, 0, 1, 0;
    -sin(Q2), -cos(Q2), 0, 0;
    0, 0, 0, 1];

T23 = [cos(Q3),-sin(Q3),0,a2;
    sin(Q3), cos(Q3),0,0;
    0, 0, 1, 0;
    0, 0, 0, 1];

T34 = [cos(Q4),-sin(Q4),0, a3;
       0,      0,       1, d4;
    -sin(Q4), -cos(Q4), 0, 0;
       0,      0,       0, 1];

T45 = [cos(Q5),-sin(Q5),0,0;
    0, 0,-1,0;
    sin(Q5), cos(Q5), 0, 0;
    0, 0, 0, 1];

T56 = [cos(Q6),-sin(Q6),0,0;
    0, 0, 1, 0;
    -sin(Q6), -cos(Q6), 0, 0;
    0, 0, 0, 1];

Ts0 = [1,0,0,0;
       0,1,0,0;
       0,0,1,d0;
       0,0,0,1];%世界坐标
   
T6t = [1,0,0,a6;
       0,1,0,0;
       0,0,1,d6;
       0,0,0,1];
   
% Res_T = simplify(Ts0*T01*T12*T23*T34*T45*T56*T6t)
% 
% L0=Link([0     169.77      0   0   0],'modified'); %定义连杆的D-H参数
% L1=Link([0     0           0    0  0],'modified'); %定义连杆的D-H参数
% L2=Link([0     0           64.2   -pi/2  0],'modified'); %定义连杆的D-H参数 a1 = 64.2
% L3=Link([0     0           305     0],'modified'); %定义连杆的D-H参数 a2 = 305
% L4=Link([0    222.63       0   -pi/2  0],'modified'); %定义连杆的D-H参数 a3 = 0
% L5=Link([0     0           0   pi/2  0],'modified'); %定义连杆的D-H参数
% L6=Link([0     36.5          0   -pi/2  0],'modified'); %定义连杆的D-H参数
% robot = SerialLink([L0 L1 L2,L3,L4,L5,L6],'name','AR2-modified');
% c=robot.fkine(Qtheta)
% 
% robot.plot(Qtheta);

resTt = Ts0*T01*T12*T23*T34*T45*T56*T6t;

T_input = resTt;
% X-Y-Z顺序==ZYX顺序
cosBeta = sqrt(T_input(1,1)*T_input(1,1)+T_input(2,1)*T_input(2,1));

%计算三个欧拉角
if(cosBeta~=0)
    EulerY = atan2(-T_input(3,1),cosBeta);
    if(EulerY > pi/2 || EulerY<-pi/2)
        cosBeta = -cosBeta;
        EulerY = atan2(-T_input(3,1),cosBeta);
    end
    EulerZ = atan2(T_input(2,1),T_input(1,1));
    EulerX = atan2(T_input(3,2),T_input(3,3));
else
    EulerY = pi/2;
    EulerZ = 0;
    EulerX = atan2(T_input(1,2),T_input(2,2));
end


resEuler = [rad2deg(EulerX),rad2deg(EulerY),rad2deg(EulerZ)];

outputXYZ = [T_input(1,4),T_input(2,4),T_input(3,4),resEuler(1),resEuler(2),resEuler(3)];