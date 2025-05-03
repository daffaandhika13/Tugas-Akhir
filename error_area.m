% Define the waypoints (your reference path)
waypoints = [
    0, 0;
    2, 1;
    4, 2;
    6, 3;
    8, 4
];

% Define the real data points (your actual path)
realData = [
    0, 0;
    2, 1.2; % Adjust these values according to your actual data
    4, 2.1;
    6, 2.9;
    8, 3.8
];

waypoints = [out.Waypoint_y(1,1:end)' out.Waypoint_x(1,1:end)'];
realData = flip(out.posisi_xy,2);


% Calculate the area formed by connecting the waypoints
waypointsArea = polyarea(waypoints(:, 1), waypoints(:, 2));

% Calculate the area formed by connecting the waypoints
waypointsArea = polyarea(waypoints(:, 1), waypoints(:, 2));

% Calculate the area formed by connecting the waypoints to the real data points
combinedArea = polyarea(realData(:, 1),  realData(:, 2));

% Calculate the error area
errorArea = abs(combinedArea - waypointsArea); % Take the absolute value

% Display the results
fprintf('Waypoints Area: %.2f square units\n', waypointsArea);
fprintf('Combined Area: %.2f square units\n', combinedArea);
fprintf('Error Area: %.2f square units\n', errorArea);

% Create a figure
figure;

hold on;

% Plot the waypoints and real data
plot( waypoints(:, 2),waypoints(:, 1), '-ro', 'LineWidth', 2, 'MarkerSize', 2);
plot( realData(:, 2),realData(:, 1), '-bo', 'LineWidth', 2, 'MarkerSize', 2);

% Plot a polygon representing the error area
errorPolygonX = [waypoints(:, 1); flip(realData(:, 1))];
errorPolygonY = [waypoints(:, 2); flip(realData(:, 2))];
fill(errorPolygonY,errorPolygonX, 'c', 'FaceAlpha', 0.5);

% Customize the plot
legend('Desired Path', 'Real Data', 'Error Area');
title('Error Area Visualization');
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
grid on;

% Optional: Save the plot to an image file
% saveas(gcf, 'error_area_plot.png');