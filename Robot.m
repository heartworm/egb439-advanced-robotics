classdef Robot < handle 
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        history = [];
        pb;
        fieldImage;
        lastPathPoint;
        lastPath;
    end
    
    methods
        function setup(self, ip, localiserIp)
            self.pb = PiBot(ip);
            self.pb.connectToLocaliser(localiserIp);
        end
        
        function pose = updatePose(self)
            pose = reshape(self.pb.getPoseFromLocaliser(), 1, []);
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
        
        function distanceToEnd = setMotionOnPath(self, path)
            followJump = 5;
            
            distances = sqrt(sum(path .^ 2, 2));
            distanceToEnd = distances(length(distances));
            [~, distanceIndices] = sort(distances);
            
            % TODO: crossovers
            
            closestIndex = distanceIndices(1);
            goalIndex = min(length(distances), closestIndex + followJump);
            goalPoint = path(goalIndex);
            self.setMotionToPose(goalPoint);
        end
        
        function setMotionToPose(self, p)
            % TODO: goal angle
            assert(length(p) == 2, 'length of point is not 2');
            
            p = reshape(p, 1, []);
            latestPose = self.getLatestPose();
            
            displacement = latestPose(1:2) - p;
            angleToPoint = atan2(displacement(2), displacement(1));
            distToPoint = norm(displacement);
            steering = angdiff(latestPose(3), angleToPoint) / pi;
            
            steeringGain = 1.5;
            speedGain = 40;
            
            self.setMotion(speedGain * distToPoint, ...
                           steeringGain * steering);
        end
        
        function setMotion(speed, steering)
            % Speed from -100 to 100 and steering from -1 to 1
            speed = max(-100, min(100, speed));
            steering = max(-1, min(1, steering));
            
            leftWheel = speed * (1 + steering);
            rightWheel = speed * (1 - steering);
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

