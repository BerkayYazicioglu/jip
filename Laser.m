%% Class for object detection sensor (modelled as a 2D laser beam)

classdef Laser < handle
    properties 
        angle_range   % range of angles [min max](degrees)
        theta_laser   % laser beam
        K             % number of beams
        max_range     % max range (m)
        std_local     % std of local measurement noise (m)
        max_noise     % probability of maximum noise
        short_noise   % probability of short noise
        placement     % placement of the sensor on agent's fixed frame (x, y, theta)
        noise = false % bool for adding noise
        measurements  % taken measurements
    end
    methods
        %% laser sensor 
        function obj = Laser(placement, angle_range, K, max_range)
            obj.placement = placement;
            obj.angle_range = angle_range;
            obj.K = K;
            obj.max_range = max_range;
            obj.theta_laser = linspace(angle_range(1), angle_range(2), K);
        end

        %% measures distances as a 2D beam
        % s: Vector that contains the laser scanning [Kx1]
        % map: map of obstacles (each row is a line, x1 y1)
        % pose: Pose [x;y;theta] of the agent
        function s = measure(obj, map, pose)
            % initialize scanning
            s = obj.max_range * ones(obj.K, 1);
            % orientation of the agent 
            theta = pose(3);
            xk = obj.placement(1);
            yk = obj.placement(2);
            thk = obj.placement(3);
            % global laser pose
            pose(1) = pose(1) + xk * cosd(theta) - yk * sind(theta);
            pose(2) = pose(2) + xk * sind(theta) + yk * cosd(theta);
            pose(3) = pose(3) + thk;
            % get location and orientation of the laser
            location = pose(1:2);
            theta = pose(3);
            angles = wrapTo180(obj.theta_laser + theta);
            
            for i = 1:obj.K
                % define the geometry of the beam
                angle = angles(i);
                % set "maximum" end point
                Tr(1) = location(1) + cosd(angle) * obj.max_range;
                Tr(2) = location(2) + sind(angle) * obj.max_range;
                % find the intersections
                [xout, yout] = polyxpoly(map(:,1), map(:,2),...
                               [location(1) Tr(1)],[location(2) Tr(2)]);
                if(isempty(xout))
                    continue
                end
                % get the min distance of intersections
                if(size(xout,1) > 1)
                    mind = inf;
                    for k = 1:size(xout,1)
                        d = norm([xout(k)-location(1),  yout(k)-location(2)]);
                        if(d < mind)
                            cxout = xout(k);
                            cyout = yout(k);
                            mind = d;
                        end
                    end
                else
                    % Just 1 intersection here
                    cxout = xout;
                    cyout = yout;
                end  
                s(i) = norm([cxout-location(1), cyout-location(2)]);
            end

            % add noise
            if(obj.noise)
                % local gaussian measurement noise 
                ind = s~= obj.max_range;
                s(ind) = s(ind) + std_local_noise*randn(size(s(ind)));
                for k=1:obj.K
                    % short readings
                    if(rand() < obj.short_noise)
                        s(k) = s(k)*rand();
                    end
                    % max readings
                    if(rand() < obj.max_noise)
                        s(k) = obj.max_range;
                    end
                end
            end
            obj.measurements = s;
        end
    end
end
