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

origin = robot.updatePose();
image = robot.updateFieldImage();

%% Test Values 
%  origin = [0,0];
% image = load('img.mat');
% image = image.img;

%% Path Planning
origin = [origin(1),origin(2)];
originUV = coordToUV(origin, const.DIM_IMAGE, const.DIM_FIELD);

[ obstacleMap, goalUV ] = DoCV(image, originUV, const.DIM_ROBOT_CELL);


dx = DXform(obstacleMap, 'inflate', round(const.DIM_ROBOT_CELL * 1.25));
dx.plan(goalUV);
pathUV = dx.query(originUV);

path = UVToCoord(pathUV, const.DIM_IMAGE, const.DIM_FIELD);



dx.plot(pathUV);
figure
hold on
axis([-1 1 -1 1]);
% 
% [obstacleV, obstacleU] = find(obstacleMap == 1);
% obstacleUV = [obstacleU, obstacleV];
% obstacleCoord = UVToCoord(pathUV, const.DIM_IMAGE, const.DIM_FIELD);

%% Motion
tic;
while(1)
    pose = robot.updatePose();
    poseUV = coordToUV(pose(1:2), const.DIM_IMAGE, const.DIM_FIELD);
    cla;
    dx.plot(pathUV);
    [pursuitIndex, distanceToEnd] = robot.setMotionOnPath(path);
    plot_circle(pathUV(pursuitIndex, :), 10);
    plot_vehicle([poseUV, pose(3)]);
%     pause(0.1);
%     refresh;
    
    if distanceToEnd < 0.1
        robot.stop();
        break;
    end
 
end

timeTaken = toc;
pathTaken = robot.history(:, 1:2);
pathLength = sum(sqrt(sum((pathTaken(1:end-1, :) - pathTaken(2:end, :)) .^ 2, 2)));
speed = pathLength / timeTaken;
fprintf('\n\nTravelled %.2fm in %2fs with avg speed %.2f m/s\n', pathLength, timeTaken, speed);



hold off