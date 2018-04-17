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

load 'map.dat';
 
 
 
%% Get image 
%img = pb.getImageFromCamera(); %save image in img
%imshow(img); %show image


%% Move Robot + Encoders
%Forward 2sec



first = 0;
last = 0;
distance = 0;
robotRadius = 0.15;
robotRadiusInd = round(0.15/1.96*49);
robotRadiusElem = ones(robotRadiusInd);
dilatedMap = idilate(map, robotRadiusElem);

posepos = [0, 0, 0];
history = [];
origin = pb.getPoseFromLocaliser();
lineHold = [origin(1), origin(2)];
goal = [round((((1+0.6)/2)) *49) ,round((((1+-0.8)/2)) *49)];
start = [round((((1+origin(1))/2)) *49) ,round((((1+origin(2))/2)) *49)];
distanceTran = distancexform(dilatedMap, goal);


[nonDriveableRows, nonDriveableCols] = find(map == 1);
nonDriveable = indexToCoord([nonDriveableRows, nonDriveableCols]);
[nonDriveableRows, nonDriveableCols] = find(dilatedMap == 1 & ~(map == 1));
nonDriveableDilated = indexToCoord([nonDriveableRows, nonDriveableCols]);


path = findPath(distanceTran, start, goal);
path = ((path / 49) * 1.96) - 1;
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
    
    distances = sqrt((goalXs - x).^2 + (goalYs - y).^2);
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