classdef Robot < handle 
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        history = [];
        pb;
        fieldImage;
        lastPathPoint = 1;
    end
    
    methods
        function setup(self, ip, localiserIp)
            self.pb = PiBot(ip);
            self.pb.connectToLocaliser(localiserIp);
        end
        
        function pose = updatePose(self)
            pose = reshape(self.pb.getPoseFromLocaliser(), 1, []);
            pose(3) = pose(3) / 180 * pi;
            self.history = [self.history; pose];
        end
        
        function pose = getLatestPose(self)
            assert(size(self.history, 1) > 0, 'There are no positions in history');
            lastRowIndex = size(self.history, 1);
            pose = self.history(lastRowIndex, :);
        end
        
        function image = updateFieldImage(self)
            image = rot90(self.pb.getImageFromLocaliser(), 2);
            self.fieldImage = image;
        end
        
        function stop(self)
            self.pb.stop();
        end
        
        function [pursuitIndex, distanceToEnd] = setMotionOnPath(self, path)
            followJump = 100;
            maxJump = 1000;
            
            currentPose = self.getLatestPose();
            vectorsToPath = path - currentPose(1:2);
            
            distances = sqrt(sum(vectorsToPath .^ 2, 2));
            distanceToEnd = distances(length(distances));
            
            [~, distanceIndices] = sort(distances);
            % TODO: crossovers
            
            closestIndex = distanceIndices(1);
            distanceToClosest = distances(closestIndex)
            minPursuitIndex = self.lastPathPoint;
            maxPursuitIndex = min(self.lastPathPoint + maxJump, length(distances));
            
            followJump = followJump + round(distanceToClosest * 0)
            
            pursuitIndex = max(minPursuitIndex, min(maxPursuitIndex, closestIndex + followJump));
            self.lastPathPoint = pursuitIndex;
            goalPoint = path(pursuitIndex, :);
            self.setMotionToPose(goalPoint);
        end
        
        function setMotionToPose(self, p)
            % TODO: goal angle
            assert(length(p) == 2, 'length of point is not 2');
            
            p = reshape(p, 1, []);
            latestPose = self.getLatestPose();
            
            displacement = p - latestPose(1:2);
            angleToPoint = atan2(displacement(2), displacement(1));
            distToPoint = norm(displacement);
            steering = angdiff(latestPose(3), angleToPoint) / pi;
            steeringGain = 0.9;
            speedGain = 200;
            
            steering = min(1, max(-1, steering * steeringGain));
            
            speed = 70;
%             speed = 40;

            self.setMotion(speed, ...
                           steeringGain * steering);
        end
        
        function setMotion(self, speed, steering)
            % Speed from -100 to 100 and steering from -1 to 1
%             speed = max(-50, min(50, speed));
            steering = max(-1, min(1, steering));
            
            leftWheel = max(-100, min(100, speed * (1 + steering)));
            rightWheel = max(-100, min(100, speed * (1 - steering)));
            
            self.pb.setMotorSpeeds(round(leftWheel), round(rightWheel));
        end
        
        function plotHistory(self)  
            plot(self.history(:,1), self.history(:,2));
        end
        
        function plotLatestPosition(self)
            plot_vehicle(self.getLatestPose());
        end    
        
        
        
    end
    
end

