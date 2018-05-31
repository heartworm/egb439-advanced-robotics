%% Constants
clear; close all; clc;
const.IP_ROBOT = '172.19.232.173';
const.DIM_ROBOT = 0.15;
const.DIM_IMAGE = 500;
const.DIM_FIELD = 1.96;
const.DIM_ROBOT_CELL = round(const.DIM_ROBOT / const.DIM_FIELD * const.DIM_IMAGE);

%% Real Values

robot = Robot();
robot.setup(const.IP_ROBOT);

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
    robot.updateImage();
    robot.updateRods();
    robot.updateStep();
    
    figure(figVideo);
    robot.drawImage();
    robot.drawRods();
    
    figure(figGraph);
    cla;
    robot.plotLatestFrame();
    robot.plotLatestRods();

    startOdo = robot.getOdometer();
    while robot.getOdometer() < startOdo + 0.1
        robot.updateMotorAngles();
        robot.updateOdometry();
        robot.predictStep();
        robot.setMotion(20, -0.5);
        disp(robot.getOdometer());
        
        figure(figGraph);
        cla;
        robot.plotLatestFrame();
        robot.plotLatestRods();
    end
    
    robot.stop();
    pause(0.25);
end

%% Release
robot.stop();