% Chukwuemeka Osaretin Ike
% ECSE 6440 - Optimal Control
% SMT-Based Optimal Controller Synthesis for a Mobile Robot

% This script plots the trajectory created by the goToGoalSMT.py
% script.

clear

load('goToGoalSMT.mat');

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

x1 = path(:,2);
y1 = path(:,4);

% Plot the simulation environment with the start point, goal and obstacle.
% Record the video.
writeObj = VideoWriter('goToGoalSMT', 'MPEG-4');
writeObj.FrameRate = 5;
open(writeObj);
figure(1);
for i=1:size(x1,1)
    plot(x1, y1,'k-.'); hold on;
    plot(x1(i), y1(i),'c>'); hold on;
    
    title('Workspace'); xlabel('X Position'); ylabel('Y Position');
    
    rectangle('Position', [initPose(2)-0.5,initPose(4)-0.5,1,1], 'FaceColor', 'none');                      % Start
    rectangle('Position', [obstacle(1), obstacle(3),obsSize], 'FaceColor', 'red');                           % Obstacle
    rectangle('Position', [-goal(2), -goal(4),stationSize], 'EdgeColor', 'green', 'FaceColor', 'none');    % Goal
    
    legend('Path', 'Robot', 'Location', 'northwest'); axis(wkSpace);
    frame = getframe(gcf);
    writeVideo(writeObj,frame);
    drawnow;
    hold off;
end
close(writeObj);