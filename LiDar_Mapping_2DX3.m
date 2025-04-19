clear;
clc; % Clear command window

% Set up serial port
port = "COM4";  % Set COM port  
baudRate = 115200; % Baud rate for UART communication
s = serialport(port, baudRate); % Create serial port object
flush(s);  % Clear the serial buffer

% Scan configuration
num_scans = 3;   % Total number of full 360° scans
values_per_scan = 32;  % Number of steps per scan (11.25° each)
angles_deg = linspace(0, 360, values_per_scan + 1); % Generate 33 angles
angles_deg(end) = [];  % Remove 360 to keep exactly 32 values
angles_rad = deg2rad(angles_deg); % Convert to radians for trig functions

% Storage arrays
x_coords = [];  % X-coordinates (depth layer)
y_coords = [];  % Y-coordinates (width, based on cosine)
z_coords = [];  % Z-coordinates (height, based on sine)


disp("Waiting for 3 complete scans...");

scan_count = 0;
% ----- Main Loop: Read Serial and Convert to 3D -----
while scan_count < num_scans
    distance_values = []; % Hold 32 distances for one scan
 % Wait for a full scan (32 distance readings)
    while length(distance_values) < values_per_scan
        if s.NumBytesAvailable > 0
            line = readline(s); % Read a line from UART
            dist = str2double(strtrim(line)); % Convert string to double

             % Only keep valid, finite numbers
            if ~isnan(dist) && isfinite(dist)
                distance_values(end+1) = dist;  % Append to current scan
                fprintf("Scan %d | Step %d | Distance: %.2f mm\n", scan_count+1, length(distance_values), dist);
            end
        end
        pause(0.01); % short wait to prevent CPU overload
    end

    % Convert polar to Cartesian (Y-Z plane, scan along X)
    y = distance_values .* cos(angles_rad);
    z = distance_values .* sin(angles_rad);
    x = ones(1, values_per_scan) * (scan_count + 1);

    % Append to full list
    x_coords = [x_coords, x];
    y_coords = [y_coords, y];
    z_coords = [z_coords, z];

    scan_count = scan_count + 1;

    fprintf("Completed scan %d of %d.\n\n", scan_count, num_scans);
end

% Reshape collected data into scan-wise matrices
x_matrix = reshape(x_coords, values_per_scan, num_scans);
y_matrix = reshape(y_coords, values_per_scan, num_scans);
z_matrix = reshape(z_coords, values_per_scan, num_scans);

% Plot the 3D connected graph
figure;
hold on;
grid on;
xlabel("X Depth");
ylabel("Y Width");
zlabel("Z Height");
title("2DX3 Project Visualization");

% Connect points in circular scans
for i = 1:num_scans
    plot3(x_matrix(:, i), y_matrix(:, i), z_matrix(:, i), '-o', 'LineWidth', 1);
end

% Connect points between scans at same angle index
for j = 1:values_per_scan
    plot3(x_matrix(j, :), y_matrix(j, :), z_matrix(j, :), 'k-', 'LineWidth', 0.5);
end

view(45, 25); % Optional: Adjust viewing angle to match your screenshot