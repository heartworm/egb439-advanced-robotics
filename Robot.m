classdef Robot < handle 
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        NOISE_ODOM = eye(2)*0.001;
        NOISE_CAMERA = [0.01, 0;
                        0,    0.0001;]
        
        ekfSigma = eye(3) * 0.0001;
        ekfMu = zeros(3,1);
        rodIndexes = containers.Map();
        history = [0,0,0];
        pb;
        fieldImage;
        image;
        imageHistory = {};
        lastPathPoint = 1;
        motorAnglesHistory = [0, 0];
        odometryHistory = [];
        rodsHistory = {};
    end
    
    methods
        function setup(self, ip)
            self.pb = PiBot(ip);
            self.pb.reset();
            self.updateMotorAngles();
            self.updateOdometry();
            self.rodIndexes = containers.Map();
        end
        
        function updateLocalisationStep(self, map)
            rods = self.getLatestRods();
            
            for i = 1:length(rods)
                rod = rods(i);
                
                mapInd = find(map(3,:) == str2double(rod.code));
                xRod = map(1:2, mapInd);
                
                x = self.ekfMu;

                delta = [xRod(1) - x(1)
                         xRod(2) - x(2)];

                range = norm(delta);
                bearing = wrapToPi(atan2(delta(2), delta(1)) - x(3));

                h = [range
                     bearing];

                z = [rod.range
                     rod.bearing];
                
                GMap = [delta(1)/range, delta(2)/range
                     -delta(2)/(range^2), delta(1)/(range^2)];
                G = [-GMap [0;-1]];
                
                K = self.ekfSigma * G' * inv(G * self.ekfSigma * G' + self.NOISE_CAMERA);
                innovation = z - h;
                innovation(2) = wrapToPi(innovation(2));
                
                self.ekfMu = self.ekfMu + K*innovation;
                I = eye(size(self.ekfSigma));
                self.ekfSigma = (I - K*G) * self.ekfSigma; 
            end 
        end
        
        function updateStep(self)
            
            rods = self.getLatestRods();
            
            for i = 1:length(rods)
                rod = rods(i);
                % if we haven't seen the rod
                if ~self.rodIndexes.isKey(rod.code) 
                    self.addRod(rod);
                    continue;
                end

                %otherwise
                ind = self.rodIndexes(rod.code);
                x = self.ekfMu(1:3);
                xRod = self.ekfMu(ind:ind+1);

                delta = [xRod(1) - x(1)
                         xRod(2) - x(2)];

                range = norm(delta);
                bearing = wrapToPi(atan2(delta(2), delta(1)) - x(3));

                h = [range
                     bearing];

                z = [rod.range
                     rod.bearing];
                
                G = zeros(2, length(self.ekfMu));
                GMap = [delta(1)/range, delta(2)/range
                     -delta(2)/(range^2), delta(1)/(range^2)];
                GRobot = [-GMap [0;-1]];
                
                G(:, ind:ind+1) = GMap;
                G(:, 1:3) = GRobot;
                
                K = self.ekfSigma * G' * inv(G * self.ekfSigma * G' + self.NOISE_CAMERA);
                innovation = z - h;
                innovation(2) = wrapToPi(innovation(2));
                
                self.ekfMu = self.ekfMu + K*innovation;
                I = eye(size(self.ekfSigma));
                self.ekfSigma = (I - K*G) * self.ekfSigma; 
            end
        end
        
        function addRod(self, rod)
            %rod shouldn't already exist.
            assert(~self.rodIndexes.isKey(rod.code));
            
            pos = self.ekfMu(1:3);
            range = rod.range;
            bearing = rod.bearing;
            ang = wrapToPi(pos(3) + bearing);
            
            ind = length(self.ekfMu) + 1;
            self.rodIndexes(rod.code) = ind;
            
            self.ekfMu = [self.ekfMu
                          pos(1) + range * cos(ang)
                          pos(2) + range * sin(ang)];
              
            jacobianMu = [cos(ang), -range*sin(ang)
                          sin(ang), range*cos(ang)];
            
            covar = jacobianMu * self.NOISE_CAMERA * jacobianMu';
            sigmaDim = length(self.ekfSigma);
            covarDim = length(covar);
            self.ekfSigma = [self.ekfSigma,     zeros(sigmaDim, covarDim)
                             zeros(covarDim, sigmaDim)  covar];           
        end
        
        function predictStep(self)
            odometry = self.getLatestOdometry();
            d = odometry(1);
            dTh = odometry(2);
            xt = self.ekfMu(1:3);
            
            model = @(xt, d, dth) [xt(1) + d * cos(xt(3))
                                   xt(2) + d * sin(xt(3))
                                   wrapToPi(xt(3) + dth)];

            
            jacobianX = [1 0 d * -sin(xt(3))
                         0 1 d * cos(xt(3))
                         0 0 1];
            
            jacobianOdom = [cos(xt(3)) 0 
                            sin(xt(3)) 0
                            0          1];
                        
            jx = eye(length(self.ekfMu));
            jo = zeros(length(self.ekfMu), 2);
            
            jx(1:3,1:3) = jacobianX;
            jo(1:3, :)  = jacobianOdom;
            
            self.ekfSigma = jx * self.ekfSigma * jx' + jo * self.NOISE_ODOM * jo';
            self.ekfMu(1:3) = model(xt, d, dTh);
        end
        
        function angles = updateMotorAngles(self)
            TICKS_PER_RADIAN = 60.467105263157900;
            angles = self.pb.getMotorEncoders() / TICKS_PER_RADIAN;
            angles = fliplr(reshape(angles, 1, []));
            self.motorAnglesHistory = [self.motorAnglesHistory; angles];
        end
        
        function ticks = getLatestMotorAngles(self)
            ticks = self.motorAnglesHistory(end, :);
        end
        
        function odometry = updateOdometry(self)
            lastAngles = self.motorAnglesHistory(end-1, :);
            newAngles = self.getLatestMotorAngles();
 
            WHEEL_RADIUS = 0.065 / 2;
            dAngles = newAngles - lastAngles;
            wheelSpeeds = dAngles * WHEEL_RADIUS;
            
            WHEEL_SPAN_RADIUS = 0.152;
            d = mean(wheelSpeeds); 
            dTh = (wheelSpeeds(2) - wheelSpeeds(1)) / WHEEL_SPAN_RADIUS;
            
            odometry = [d, dTh];
            
            self.odometryHistory = [
                self.odometryHistory;
                odometry
            ];
        end
        
        function odometer = getOdometer(self)
            odometer = sum(self.odometryHistory(:,1));
        end
        
        function odometry = getLatestOdometry(self)
            odometry = self.odometryHistory(end, :);
        end
        
        function pose = getLatestPose(self)
            pose = self.ekfMu(1:3)';
        end
        
        function image = updateImage(self)
            image = fliplr(rot90(self.pb.getImageFromCamera()));
            self.imageHistory{end + 1} = image;
        end
        
        function image = getLatestImage(self)
            image = self.imageHistory{end};
        end
        
        function rods = updateRods(self)
            pose = self.getLatestPose();
            poseTrans = transl2(pose(1), pose(2)) * trot2(pose(3));
            rods = findRods(self.getLatestImage(), poseTrans);
            self.rodsHistory{end + 1} = rods;
        end
        
        function rods = getLatestRods(self)
            rods = self.rodsHistory{end};
        end

        function drawImage(self)
            idisp(self.getLatestImage());
        end
        
        function stop(self)
            self.pb.stop();
        end
        
        function drawRods(self)
            rods = self.rodsHistory{end};
            for i = 1:length(rods)
                rod = rods(i);
                tl = rod.box(1,:);
                br = rod.box(2,:);
                line([tl(1), tl(1), br(1), br(1), tl(1)], ...
                     [tl(2), br(2), br(2), tl(2), tl(2)]);

                text(tl(1), tl(2), sprintf('Code: %s\nRange (m): %.2f\nBearing (deg): %.2f', ...
                                           rod.code, rod.range, rad2deg(rod.bearing)));
            end
        end
        
        function plotRods(self)
            rods = cell2mat(self.rodsHistory);
            plot([rods.x], [rods.y]);
        end
        
        function [pursuitIndex, distanceToEnd] = setMotionOnPath(self, path)
            followJump = 20;
            maxJump = 20;
            
            currentPose = self.getLatestPose();
            vectorsToPath = path - currentPose(1:2);
            
            distances = sqrt(sum(vectorsToPath .^ 2, 2));
            distanceToEnd = distances(length(distances));
            
            
            minPursuitIndex = self.lastPathPoint;
            maxPursuitIndex = min(self.lastPathPoint + maxJump, length(distances));
            
            
            [~, closestIndices] = sort(distances);
            
            closestIndices = closestIndices(closestIndices < maxPursuitIndex);
            
            % TODO: crossovers
            
            closestIndex = closestIndices(1);
            pursuitIndex = max(minPursuitIndex, min(maxPursuitIndex, closestIndex + followJump));
            self.lastPathPoint = pursuitIndex;
            goalPoint = path(pursuitIndex, :);
            self.setMotionToPose(goalPoint);
        end
        
        function distToPoint = setMotionToPose(self, p)
            % TODO: goal angle
            assert(length(p) == 2, 'length of point is not 2');
            
            p = reshape(p, 1, []);
            latestPose = self.getLatestPose();
            
            displacement = p - latestPose(1:2);
            angleToPoint = atan2(displacement(2), displacement(1));
            distToPoint = norm(displacement);
            steering = angdiff(latestPose(3), angleToPoint) / pi;
            steeringGain = 1.25;
            speedGain = 40;
            
            steering = min(1, max(-1, steering * steeringGain));            

            speed = round(distToPoint * speedGain);

            speed = max(20, speed);
            
            self.setMotion(speed, ...
                           steeringGain * steering);
        end
        
        function setMotion(self, speed, steering)
            % Speed from -100 to 100 and steering from -1 to 1
%             speed = max(-50, min(50, speed));
            steering = max(-1, min(1, steering));
            speed = max(-100, min(100, speed));
            
            leftWheel = max(-100, min(100, speed * (1 + steering)));
            rightWheel = max(-100, min(100, speed * (1 - steering)));
            
            self.pb.setMotorSpeeds(round(rightWheel), round(leftWheel));
        end
        
        function plotHistory(self)  
            plot(self.history(:,1), self.history(:,2));
        end
        
        function plotLatestPosition(self)
            plot_vehicle(self.getLatestPose());
        end    
        
        function plotLatestRods(self)
            codes = self.rodIndexes.keys();
            for i = 1:length(codes)
                code = codes{i};
                rodIndex = self.rodIndexes(code);
                rodPosition = self.ekfMu(rodIndex:rodIndex+1);
                plot(rodPosition(1), rodPosition(2), 'r*');
                
                covar = self.ekfSigma(rodIndex:rodIndex+1, rodIndex:rodIndex+1);
                error_ellipse(covar, rodPosition);
            end
        end
        
        function plotLatestFrame(self)
            length = 0.1;
            pose = self.getLatestPose();
            
            to_coord = @(mat) mat(1:2, 3)';
            
            transform = transl2(pose(1), pose(2)); 
            centre = transl2(pose(1), pose(2)) * trot2(pose(3));
            front = centre * transl2(length, 0);
            left = centre * transl2(0, length);
            x_line = [to_coord(centre); to_coord(front)];
            y_line = [to_coord(centre); to_coord(left)];
            line(x_line(:,1), x_line(:,2), 'Color', 'red');
            line(y_line(:,1), y_line(:,2), 'Color', 'green');
            
            covar = self.ekfSigma(1:2,1:2);
            error_ellipse(covar, self.ekfMu(1:2));
        end
    end
    
end

