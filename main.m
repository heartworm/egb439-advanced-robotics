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
goal = [-0.6, 0.4];

%% Test Values 
%  origin = [0,0];
% image = load('img.mat');
% image = image.img;

%% Motion

figure();
hold on;
axis equal;
axis([-1 1 -1 1]);
grid minor;
plot(goal(1), goal(2), 'pentagram');
while(1)
    pose = robot.updatePose();
    robot.plotLatestFrame();
    pause(0.001);
    distToGoal = robot.setMotionToPose(goal);
    if distToGoal < 0.1
        break;
    end
end

figure();
rods = findRods(robot.updateImage())

robot.stop();