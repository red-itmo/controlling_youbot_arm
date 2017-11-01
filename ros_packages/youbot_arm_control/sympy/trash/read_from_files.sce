clc();
clear;
N = 21;
for i = 1:N
    name = "/media/data/evo/robotics_report/ros_packages/youbot_arm_control/sympy/EE/EE" + string(i-1) + ".txt";
    r = read(name, -1, 5);
    EE1(:,:,i) = r'
end
