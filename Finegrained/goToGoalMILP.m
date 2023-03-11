% Rensselaer Polytechnic Institute - Julius Lab
% ARM Project
% Author - Chukwuemeka Osaretin Ike
%
% Description:
% We formulate and solve the following MILP model:
% minimize
%             sum of input energy,
% subject to
%             system dynamics, and
%             MTL specification of the task.
% The optimization variables are - u1 u2 x1 x2 y1 y2 z1 z2 z3 z4.
% There is an optimization variable for each u1(k), u2(k), etc.
% i.e. for a horizon of 50, we have (10 variables) * (50 iterations) + (4
% states) = 504 variables.
% Variables are ordered [u11, u21, u12, u22, ..., x11, x21, y11, y21, ...].

clear

% Set the simulation horizon for the variables.
u_horizon = 100;
z_horizon = u_horizon;
x_horizon = u_horizon + 1;

% Number of columns. There is one column for each variable at each 
% time step.
nu = 2;
nz = 4;
nx = 4;
col = nu*u_horizon + nz*z_horizon + nx*x_horizon;

% Workspace.
wkSpace = [0 200 0 250]';

% Initial pose.
initPose = [0 50 0 50]';

% Obstacle.
num_obs_p = 4;
obstacle = [60 -70 40 -70]';
obsSize = [-obstacle(2)-obstacle(1) -obstacle(4)-obstacle(3)];
% obstacle = [5 -6 2.5 -9]';

% % The station we will be visiting.
% station = [40 60 180 220]';
% station = [2 -3 8 -9]';
stationSize = [20 40];

% Goal position.
goal = [120 -100 100 -60]';
% goal = [9 -8 9 -8]';

% Time step for discretization.
dt = 0.1;
bi = (dt^2)/2;

% Big M parameter.
M = -10^10;


% Equality constraints for initial state.
A_init = zeros(1, col);
A_init(1:4, nu*u_horizon+(1:4)) = eye(4);
b_init = initPose;


% Equality constraints for the state transitions.
A_eq = zeros(nx*u_horizon, col);
b_eq = zeros(nx*u_horizon, 1);

for i = 1:u_horizon
    % x.
    A_eq(nx*i-3:nx*i-2, 2*i-1) = [-dt; -bi];
    A_eq(nx*i-3:nx*i-2, nu*u_horizon+1+nx*(i-1):nu*u_horizon+2+nx*(i-1)) = [-1 0; -dt -1];
    A_eq(nx*i-3, nu*u_horizon+1+nx*i) = 1;
    A_eq(nx*i-2, nu*u_horizon+2+nx*i) = 1;

    % y.
    A_eq(nx*i-1:nx*i, 2*i) = [-dt; -bi];
    A_eq(nx*i-1:nx*i, nu*u_horizon+3+nx*(i-1):nu*u_horizon+4+nx*(i-1)) = [-1 0; -dt -1];
    A_eq(nx*i-1, nu*u_horizon+3+nx*i) = 1;
    A_eq(nx*i, nu*u_horizon+4+nx*i) = 1;

    % RHS of the constraint.
    b_eq(nx*i-3:nx*i, 1) = [0 0 0 0]';
end


% Inequality constraints for obstacle avoidance.
A_obs = zeros(nx*u_horizon, col);
b_obs = zeros(nx*u_horizon, 1);

for i = 1:u_horizon-1
    % x.
    A_obs(nx*i-3:nx*i-2, 2*i-1) = [bi -bi]';                          
    A_obs(nx*i-3:nx*i-2, nu*u_horizon+1+nx*(i-1):nu*u_horizon+2+nx*(i-1)) = [dt 1; -dt -1];
    
    % y.
    A_obs(nx*i-1:nx*i, 2*i) = [bi -bi]';
    A_obs(nx*i-1:nx*i, nu*u_horizon+3+nx*(i-1):nu*u_horizon+4+nx*(i-1)) = [dt 1; -dt -1];
    
    % Weight for each of the four binary decision variables.
    A_obs(nx*i-3, nu*u_horizon + nx*x_horizon+1+nx*(i-1)) = M;
    A_obs(nx*i-2, nu*u_horizon + nx*x_horizon+2+nx*(i-1)) = M;
    A_obs(nx*i-1, nu*u_horizon + nx*x_horizon+3+nx*(i-1)) = M;
    A_obs(nx*i, nu*u_horizon + nx*x_horizon+4+nx*(i-1)) = M;
    
    % RHS of the constraint.
    b_obs(nx*i-3:nx*i, 1) = obstacle - 2; % Padding to obstacle.
end

% Inequality constraints for z_k.
A_z = zeros(u_horizon, col);
b_z = zeros(u_horizon, 1);
for i = 1:u_horizon
    A_z(i, nu*u_horizon+nx*x_horizon+1+nx*(i-1):nu*u_horizon+nx*x_horizon+4+nx*(i-1)) = [1 1 1 1];
    b_z(i, 1) = 3;
end

% Inquality constraints for final state.
% x.
A_fin = zeros(nx, col);
A_fin(1:2, 2*u_horizon-1) = [bi -bi]';
A_fin(1:2, 2*u_horizon+nx*u_horizon-3:2*u_horizon+nx*u_horizon-2) = [dt 1; -dt -1];

% y.
A_fin(3:4, 2*u_horizon) = [bi -bi]';
A_fin(3:4, 2*u_horizon+nx*u_horizon-1:2*u_horizon+nx*u_horizon) = [dt 1; -dt -1];

% RHS of the constraint.
b_fin(1:4, 1) = goal;


A = [A_eq; A_obs; A_z; A_fin; A_init];% A_sta; A8; A_z; A_fin; A_init];
b = [b_eq; b_obs; b_z; b_fin; b_init];% b_sta; b8; b_z; b_fin; b_init];

% Define the 'sense' (inequality or equality conditions).
sense = [
    repmat('=', 1, size(A_eq, 1)),...
    repmat('<', 1, size(A_obs, 1)),...
    repmat('<', 1, size(A_z, 1)),...
    repmat('<', 1, size(A_fin, 1)),...
    repmat('=', 1, size(A_init, 1))...
];


% Define the objective cost matrix.
objective = zeros(col);
for i = 1:2*u_horizon
    objective(i,i) = 1;
end

% Upper and Lower Bounds for the states.
ub_states = 100*ones(1, nx*x_horizon);
lb_states = -100*ones(1, nx*x_horizon);

% Upper and Lower Bounds for the inputs.
ub_input = repmat(100, 1, nu*u_horizon);
lb_input = -ub_input;

% Upper and Lower Bounds for z1-z4.
ub_bin = ones(1, nz*z_horizon);
lb_bin = zeros(1, nz*z_horizon);

% Concatenate all upper and lower bound matrices.
lb = [lb_states, lb_input, lb_bin];
ub = [ub_states, ub_input, ub_bin];

% Define the variable types for the solver.
uType = repmat('C', 1, nu*u_horizon);
xType = repmat('C', 1, nx*x_horizon);
zType = repmat('B', 1, nz*z_horizon);
varType = [uType xType zType];

% Solve the MILP.
startTime = clock;
try
    clear model;
    model.A = sparse(A);
    model.obj = zeros(1,col);
    model.Q = sparse(objective);
    model.rhs = b;
    model.sense = sense;
    model.vtype = varType;
    model.lb = lb;
    model.ub = ub;
    
    clear params;
    params.outputflag = 0;
    params.DualReductions = 0;
    
    result = gurobi(model, params);

catch gurobiError
    fprintf('There has been an error\n');
end
endTime = clock;
fprintf("Runtime: %4.4f\n", etime(endTime, startTime))

% Read out the solution. We need x2 and y2 through the horizon.
x2 = zeros(x_horizon,1);
y2 = zeros(x_horizon,1);
j = 1;
for i = nu*u_horizon + (2:nx:nx*x_horizon)
    x2(j) = result.x(i);
    y2(j) = result.x(i+2);
    j = j+1;
end
minJ = result.objval;

% Plot the simulation environment with the start point, goal and obstacle.
% Record the video.
% writeObj = VideoWriter('goToGoalMILP');
% writeObj.FrameRate = 5;
% open(writeObj);
figure(1);
for i=1:size(x2,2)
    plot(x2, y2,'k-.'); hold on;
    plot(x2(i), y2(i),'c>'); hold on;
    
    title('Workspace'); xlabel('X Position'); ylabel('Y Position');
    
    rectangle('Position', [initPose(2)-0.5,initPose(4)-0.5,1,1], 'FaceColor', 'none');                      % Start
    rectangle('Position', [obstacle(1), obstacle(3),obsSize], 'FaceColor', 'red');                           % Obstacle
    rectangle('Position', [-goal(2), -goal(4),stationSize], 'EdgeColor', 'green', 'FaceColor', 'none');    % Goal
    
    legend('Path', 'Robot', 'Location', 'northwest'); axis(wkSpace);
    
%     frame = getframe(gcf);
%     writeVideo(writeObj,frame);
    drawnow; hold off;
end