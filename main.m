%% Constants
clear; close all; clc;
const.IP_ROBOT = '172.19.232.173';
const.DIM_ROBOT = 0.15;
const.DIM_IMAGE = 500;
const.DIM_FIELD = 1.96;
const.DIM_ROBOT_CELL = round(const.DIM_ROBOT / const.DIM_FIELD * const.DIM_IMAGE);

%% Path
angles = linspace(0, 4*pi, 200)';
path = 0.3 * [cos(angles) sin(angles)];

%% Real Values
robot = Robot();
robot.setup(const.IP_ROBOT);

%% Visualisation
figVideo = figure('OuterPosition', [50,100,750,750]);
figGraph = figure('OuterPosition', [800,100,750,750]);
hold on;
axis equal;
axis([-1 1 -1 1]);
grid minor;
% plot(goal(1), goal(2), 'pentagram');
task = 1;
pursuitIndex = 1;
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
    plotPath(path, pursuitIndex);

    startOdo = robot.getOdometer();
    if task == 1 || task == 2

        while robot.getOdometer() < startOdo + 0.05
            robot.updateMotorAngles();
            robot.updateOdometry();
            robot.predictStep();

            figure(figGraph);
            cla;
            robot.plotLatestFrame();
            robot.plotLatestRods();

            if task == 1
                [pursuitIndex, distance] = robot.setMotionOnPath(path);
                plotPath(path, pursuitIndex);
                if pursuitIndex == length(path) && distance < 0.1
                    robot.stop();
                    task = 2;
                end
            elseif task == 2
                allPositions = reshape(robot.ekfMu(4:end), 2, []);
                meanPosition = mean(allPositions, 2);
                distance = robot.setMotionToPose(meanPosition);
                plot(meanPosition(1), meanPosition(2), 'pentagram');
                if distance < 0.1
                    task = 3;
                    break;
                end
            end
        end
    elseif task == 3
        robot.stop();
        fprintf("The drone was located at %f, %f\n", meanPosition(1), meanPosition(2));
        break;
    end
        

    robot.stop();
    pause(0.25);
end

%% Release
robot.stop();


function plotPath(path, ind)
    plot(path(:,1), path(:,2));
    plot(path(ind,1), path(ind,2), 'ro');
end