%% Constants
clear; close all; clc;
const.IP_ROBOT = '172.19.232.102';
const.IP_LOCALISER = '172.19.232.12';
const.DIM_ROBOT = 0.15;
const.DIM_IMAGE = 500;
const.DIM_FIELD = 1.96;
const.DIM_ROBOT_CELL = round(const.DIM_ROBOT / const.DIM_FIELD * const.DIM_IMAGE);

%% Real Values

robot = Robot();
robot.setup(const.IP_ROBOT, const.IP_LOCALISER);

%% Visualisation
figVideo = figure();
figGraph = figure();
hold on;
axis equal;
axis([-1 1 -1 1]);
grid minor;
% plot(goal(1), goal(2), 'pentagram');

%% Motion
while(1)
    pose = robot.updatePose();
    robot.updateImage();
    rods = robot.updateRods();
    
    figure(figGraph);
    robot.plotLatestFrame();
    plot([rods.x], [rods.y], 'r*');
    
    figure(figVideo);
    robot.drawImage();
    robot.drawRods();
    
    pause(0.001);
    robot.setMotion(40,-0.25);
    pause(0.5); 
    robot.stop();
    pause(0.5);
end

%% Release
robot.stop();