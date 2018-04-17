%% Enter IP address shown on robot
clear;
close all;
clc;
IP = '172.19.232.102';

%Initialise robot
% pb = PiBot(IP);

if ~exist('pb')
    pb = PiBot(IP);
end

pb.connectToLocaliser('172.19.232.12');

 
 
%% Get image 
%img = pb.getImageFromCamera(); %save image in img
%imshow(img); %show image


%% Move Robot + Encoders
%Forward 2sec
   `
robotRadius = 0.15;
imageSideDim = 500;
robotRadiusInd = round(0.15/1.96*imageSideDim);
robotRadiusElem = ones(robotRadiusInd);

posepos = [0, 0, 0];
history = [];
origin = pb.getPoseFromLocaliser();
lineHold = [origin(1), origin(2)];

startIndices = coordToIndex(reshape(origin(1:2), 1, []));

image = fliplr(flipud(pb.getImageFromLocaliser()));
[obstacleMap, goalIndices] = DoCV(image);
dilatedMap = idilate(obstacleMap, robotRadiusElem);

% dx = DXform(obstacleMap);
% dx.plan(goalIndices);
% dx.query(start);

distanceTran = distancexform(dilatedMap, fliplr(goalIndices));

% for display purposes
[nonDriveableRows, nonDriveableCols] = find(obstacleMap == 1);
nonDriveable = indexToCoord([nonDriveableRows, nonDriveableCols]);
[nonDriveableRows, nonDriveableCols] = find(dilatedMap == 1 & ~(dilatedMap == 1));
nonDriveableDilated = indexToCoord([nonDriveableRows, nonDriveableCols]);
%%%%%%%%%%%%%%%%%5

path = findPath(distanceTran, startIndices, goalIndices);
path = indexToCoord(path);
% path(2) = path(2) * -1;

goalXs = path(:,1);
goalYs = path(:,2);
pathSize = size(path);
pathLength = pathSize(1);

figure 
hold on
axis([-1,1, -1,1]);

followDist = 5;
maxJump = 15;

goalIndex = -1;

while (1)
    posepos = pb.getPoseFromLocaliser();
    disp(posepos);

    x = posepos(1);
    y = posepos(2);
    th = posepos(3) / 180 * pi;
    
    distances = sqrt();
    distanceToEnd = distances(length(distances));
    
    [~, indices] = sort(distances);
    indices = indices((indices - goalIndex) < maxJump);
    
    if distanceToEnd < 0.1
        break;
    end
    
    disp(indices);
    
    newGoalIndex = indices(1) + followDist;
        
    goalIndex = min(newGoalIndex, pathLength);
    
    disp(goalIndex);
    
    goalX = goalXs(goalIndex);
    goalY = goalYs(goalIndex);
    
    %plotting
    cla;
    plot(goalXs, goalYs, 'r--');
    plot(0.6, -0.8, 'pentagram');
    plot(goalX, goalY, 'o');
    plot(nonDriveable(:,1), nonDriveable(:,2), 'bo');
    plot(nonDriveableDilated(:,1), nonDriveableDilated(:,2), 'ko');
    plot(origin(1), origin(2), 'yo');
    plot_vehicle([x,y,th]);
    %end
    
    
    
    lineHold = [lineHold; x, y];
    
    angleToO = atan2(goalY-y,goalX-x);
    steering = angdiff(th, angleToO) / pi;
    dist = distances(goalIndex);
    
    steeringGain = 1.5;
    steering = max(-1, min(1, steering * steeringGain));
    speedGain = 40;
    speed = max(20, min(50, dist * speedGain));
    leftWheel = speed * (1 + steering);
    rightWheel = speed * (1 - steering);
    
    pb.setMotorSpeeds(round(leftWheel), round(rightWheel));
end
hold off;

%Stop
pb.stop();