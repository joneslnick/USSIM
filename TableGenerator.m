
close all;
clear; 
clc;

% Generate Lookup Tables

low1 = -10;
high1 = 10;
low2 = -2048;
high2 = 2047;

AZ_lim = [-62, 62];
EL_lim = [-1.5, 29.5];
BAZ_lim = [-42, 42];
HRAZ_lim = [-42, 42];


f = 78.125e3; % Given carrier frequency
w = 2 * pi * f;
dac = @(value, input_low, input_high, output_low, output_high) round( output_low + (value - input_low) * (output_high - output_low) / (input_high - input_low) );

line_length = 10;

sample_num = 100;


t = linspace(0, sample_num);

carrier_wave = sin(w*t);

figure(1);
plot(t, carrier_wave, 'LineWidth', 2);
grid on;

%% Standard Sine Wave at Carrier Frequency
fprintf('Carrier Signal Lookup Table\n');
for k = 1:sample_num/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(carrier_wave(line_length*(k - 1) + m ), low1, high1, low2, high2) );
    end
    fprintf('\n');
end

fprintf('\n\n');

%% Azimuth Sinc Pulse
ang_min = AZ_lim(1);
ang_max = AZ_lim(2);
bw = 2;
a = 10;

t = linspace(0, 50*(ang_max - ang_min));
theta_t = ang_min + (t ./ 50);
theta_r = -10;

x = (theta_t - theta_r) / (1.15 * bw);

azimuth = a * (sin(pi * x) ./ (pi * x));

figure(2);
plot(theta_t, azimuth, 'LineWidth', 2);
title('Azimuth');
xlabel('\theta(t)');
ylabel('Voltage');
grid on;

fprintf('AZ Scanning Beam Lookup Table\n');
for k = 1:length(azimuth)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(azimuth(line_length*(k-1) + m), low1, high1, low2, high2));
    end
    fprintf('\n');
end

fprintf('\n\n');

%% Elevation Sinc Pulse
ang_min = EL_lim(1);
ang_max = EL_lim(2);
bw = 1.5;
a = 10;

t = linspace(0, 50*(ang_max - ang_min));
theta_t = ang_min + (t ./ 50);
theta_r = 3;

x = (theta_t - theta_r) / (1.15 * bw);

elevation = a * (sin(pi * x)) ./ (pi * x);

figure(3);
plot(theta_t, elevation, 'LineWidth', 2);
title('Elevation');
xlabel('\theta(t)');
ylabel('Voltage');
grid on;

fprintf('EL Scanning Beam Lookup Table\n');
for k = 1:length(elevation)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(elevation(line_length*(k-1) + m), low1, high1, low2, high2));
    end
    fprintf('\n');
end

fprintf('\n\n');

%% Back Azimuth Sinc Pulse
ang_min = BAZ_lim(1);
ang_max = BAZ_lim(2);
bw = 2;
a = 10;

t = linspace(0, 50*(ang_max - ang_min));
theta_t = ang_min + (t ./ 50);
theta_r = -10;

x = (theta_t - theta_r) / (1.15 * bw);

backazimuth = a * (sin(pi * x)) ./ (pi * x);

figure(4);
plot(theta_t, backazimuth, 'LineWidth', 2);
title('Back-Azimuth');
xlabel('\theta(t)');
ylabel('Voltage');
grid on;

fprintf('BAZ Scanning Beam Lookup Table\n');
for k = 1:length(backazimuth)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(backazimuth(line_length*(k-1) + m), low1, high1, low2, high2));
    end
    fprintf('\n');
end
