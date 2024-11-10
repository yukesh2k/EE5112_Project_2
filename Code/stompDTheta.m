% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);
% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% TODO: iterate over all joints to compute dtheta: (complete your code according to the STOMP algorithm) 
for j = 1:nJoints
    % Iterate over each time step in the trajectory
    for t = 1:nDiscretize
        % Calculate dtheta for joint j at time step t
        % Sum of weighted differences across all samples
        dtheta(j, t) = sum(trajProb(:, t) .* em{j}(:, t));
    end
end

end