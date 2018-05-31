%% Constants
clear; close all; clc;
const.IP_ROBOT = '172.19.232.173';
const.DIM_ROBOT = 0.15;
const.DIM_IMAGE = 500;
const.DIM_FIELD = 1.96;
const.DIM_ROBOT_CELL = round(const.DIM_ROBOT / const.DIM_FIELD * const.DIM_IMAGE);

%% Real Values
load module4_prac1.mat;
unload1(3) = wrapToPi(deg2rad(unload1(3)));
unload2(3) = wrapToPi(deg2rad(unload2(3) + 15));
unload1(1:2) = unload1(1:2) + [-0.1; 0.02];
unload2(1:2) = unload2(1:2) + [-0.1; -0.02];
% goals = [unload1, unload2];
goals = [unload1, [-0.25;0.25;wrapToPi(3*pi/2)], unload2];
robot = Robot();
robot.setup(const.IP_ROBOT);

%% Visualisation
figVideo = figure('OuterPosition', [100,100,500,500]);
figGraph = figure('OuterPosition', [600,100,500,500]);
hold on;
axis equal;
axis([-1 1 -1 1]);
grid minor;
% plot(goal(1), goal(2), 'pentagram');

task = 0;
goal = 1;
%% Motion
while(1)
    robot.updateImage();
    robot.updateRods();
    robot.updateLocalisationStep(map);
    
    figure(figVideo);
    robot.drawImage();
    robot.drawRods();
    
    figure(figGraph);
    cla;
    robot.plotLatestFrame();
    plotMap(map, goals);
%     robot.plotLatestRods();

    if task == 0 % seeking a point
        startOdo = robot.getOdometer();
        while robot.getOdometer() < startOdo + 0.05
            robot.updateMotorAngles();
            robot.updateOdometry();
            robot.predictStep();
            distance = robot.setMotionToPose(goals(1:2, goal));
            disp(distance);
            figure(figGraph);
            cla;
            robot.plotLatestFrame();
            plotMap(map, goals);

            if distance < 0.02
                task = 1;
                break;
            end
        end 
    elseif task == 1 % turn on spot
        robot.stop();
        pos = robot.getLatestPose();
        while abs(pos(3) - goals(3, goal)) > deg2rad(5)
            robot.updateMotorAngles();
            robot.updateOdometry();
            robot.predictStep();
            robot.pb.setMotorSpeeds(10, -10);
            
            figure(figGraph);
            cla;
            robot.plotLatestFrame();
            plotMap(map, goals);         
           
            pos = robot.getLatestPose();
            
            disp(abs(pos(3)-goals(3, goal)));
        end
        robot.stop();
        
        if goal ~= 2
            pause(5);
        end
        
        if goal == 3
            break;
        end
        
        goal = goal + 1;
        task = 0;
    end

    robot.stop();
    pause(0.25);
end

%% Release
robot.stop();

function plotMap(map, goals)
    plot(map(1,:), map(2,:), 'r*');
    plot(goals(1,:), goals(2,:), 'pentagram');
end