%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics
    

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
% Because the getTranform() function can only takes in structure array
% robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
[tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% get the number of joints
nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 4); 

for k=1:nJoints
    % get the homegeneous transformation from kth joint's frame to the
    % base frame
    % Use the Matlab built-in function getTransfrom to obtain the pose T
    % getTransform can only takes in structure array Configuration
    %% TODO:
    T{k} = getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k});
    % Get joint's world coordinates
    X(k, :) = [T{k}(1:3, 4)', 1]; % the position in world frame, appended with 1
end
    
end

function T = getTransform_POE(robot_struct, tConfiguration, bodyName)
    % Find the index of the joint based on the body name
    bodyIndex = find(strcmp(robot_struct.BodyNames, bodyName));
    
    if isempty(bodyIndex)
        error('Body with name %s not found in the robot structure.', bodyName);
    end
    
    % Define the home configuration transformation matrix of the base
    T_base = eye(4);  % Assuming identity for the base frame
    
    % Define screw axes for each joint in the robot
    % Format: [wx, wy, wz, vx, vy, vz] for each joint
    screw_axes = robot_struct.ScrewAxes;
    
    % Initialize the transformation matrix to identity
    T = T_base;

    % Loop through each joint up to the specified joint (bodyIndex) to apply POE
    for i = 1:bodyIndex
        % Get the screw axis for the i-th joint
        S = screw_axes(:, i);  % Assume screw_axes is a 6xN matrix

        % Decompose screw axis into angular and linear components
        w = S(1:3);  % Rotation axis part of the screw axis
        v = S(4:6);  % Translation part of the screw axis

        % Get the joint angle from the configuration
        theta_i = tConfiguration(i).JointPosition;

        % Compute the transformation for the i-th joint using POE
        T = T * expm(twistMatrix(w, v) * theta_i);
    end
end

% Helper function to create the twist matrix
function twist_mat = twistMatrix(w, v)
    % Create the twist matrix for a given screw axis
    twist_mat = [skew(w), v; 0, 0, 0, 0];
end

% Helper function to create the skew-symmetric matrix of a vector
function S = skew(w)
    % Create a skew-symmetric matrix from a vector
    S = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
end