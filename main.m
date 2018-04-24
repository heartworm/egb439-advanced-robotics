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
robot.pb.setMotorSpeeds(20,10);

%% Test Values 
%  origin = [0,0];
% image = load('img.mat');
% image = image.img;

%% Motion
figure();
axis([-1 1 -1 1]);
while(1)
    pose = robot.updatePose();
    plot_vehicle(pose);
    distToGoal = robot.setMotionToPose([0.5,-0.25]);
    if distToGoal < 0.1
        break;
    end
end
robot.stop();