%% Constants
clear; close all; clc;
const.IP_ROBOT = '172.19.232.102';
const.IP_LOCALISER = '172.19.232.12';
const.DIM_ROBOT = 0.15;
const.DIM_IMAGE = 500;
const.DIM_FIELD = 1.96;
const.DIM_ROBOT_CELL = const.DIM_ROBOT / const.DIM_FIELD * const.DIM_IMAGE;

%% Real Values
robot = Robot();
robot.setup(const.IP_ROBOT, const.IP_LOCALISER);

origin = robot.updatePose();
image = robot.updateFieldImage();

%% Test Values 
% origin = [0,0];
% image = load('img.mat');
% image = image.img;

%% Path Planning
originUV = coordToUV(origin, const.DIM_IMAGE, const.DIM_FIELD);

[ obstacleMap, goalUV ] = DoCV(image);

dx = DXform(obstacleMap, 'dilate', const.DIM_ROBOT_CELL);
dx.plan(goalUV);
pathUV = dx.query(originUV);

path = UVToCoord(pathUV, const.DIM_IMAGE, const.DIM_FIELD);

%% Motion
while(1)
    robot.updatePose();
    
    dx.plot();
    robot.plotHistory();
    robot.plotLatestPosition();
    
    distanceToEnd = robot.setMotionOnPath(path);
    if distanceToEnd < 0.1
        robot.stop();
        break;
    end
end

