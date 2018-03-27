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
radius = 2;
posepos = [0, 0, 0];
history = [];
origin = pb.getPoseFromLocaliser();
lineHold = [origin(1), origin(2)];

goalX = round((1-((1+0.6)/2)) *49);
goalY = round((1-((1+-0.8)/2)) *49);
distanceTran = distancexform(map, [goalX,goalY]);

originX = round((1-((1+origin(1))/2)) *49);
originY = round((1-((1+origin(2))/2)) *49);

current = [originX, originY];

goalXs = originX;
goalYs = originY;

while(distanceTran(current(1), current(2)) ~= 0)
    shortest = distanceTran(current(1), current(2));
    shortestX = current(1); shortestY = current(2);
    for i = -1:1
        for j = -1:1
            if distanceTran((shortestX(1)+i), shortestY(2)+j) < shortest
                shortest = distanceTran((shortestX(1)+i), shortestY(2)+j);
                shortestX = (shortestX(1)+i);
                shortestY = (shortestY(2)+j);
            end
        end
    end
    current = [shortestX, shortestY];
    goalXs = [goalXs; shortestX];
    goalYs = [goalYs; shortestY];
    
end

goalXs = 1-((1-(goalXs ./ 49))./2);
goalYs = 1-((1-(goalYs ./ 49))./2);


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
    
    
    [~, indices] = sort(distances);
    indices = indices((indices - goalIndex) < maxJump);
    
    
    disp(indices);
    
    newGoalIndex = indices(1) + followDist;
    
    if newGoalIndex > goalIndex
        goalIndex = mod(newGoalIndex - 1, length(goalXs)) + 1;
    end
    
    disp(goalIndex);
    
    goalX = goalXs(goalIndex);
    goalY = goalYs(goalIndex);
    
    %plotting
    cla;
    plot(goalXs, goalYs, 'r--');
    plot(goalX, goalY, 'pentagram   ');
    plot(originX, originY);
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