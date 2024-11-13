function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% TODO: complete the following code according to Eq (13) in the STOMP conference paper.
% try
    cost_array = zeros(length(sphere_centers), 1);  % Initialize cost array
    
    % Loop through each sphere center
    for i = 1:length(sphere_centers)
        % Get the signed distance from the voxel world to the sphere center
        d_xb = voxel_world_sEDT(idx(i,1), idx(i,2), idx(i,3));  % Distance to the obstacle in voxel grid
        
        % Calculate the cost based on the formula
        rb = radius(i);  % Radius of the obstacle
        cost = max(safety_margin + rb - d_xb, 0) * norm(vel(i,:));  % Cost function for this obstacle
        
        % Accumulate the cost
        cost_array(i) = cost;
    end
    cost = sum(cost_array);
% catch  % for debugging
%     disp('in debug');
%     idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end