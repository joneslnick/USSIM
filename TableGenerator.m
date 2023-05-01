
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

line_length = 8;

sample_rate = 16;


t = linspace(0,1/f, sample_rate);

carrier_wave = sin(w*t);

figure(1);
plot(t, carrier_wave, 'LineWidth', 2);
grid on;

%% Standard Sine Wave at Carrier Frequency
fprintf('Carrier Signal Lookup Table\n');
for k = 1:sample_rate/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(carrier_wave((k - 1) + m ), low1, high1, low2, high2) );
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

azimuth = a * (sin(pi * x) ./ (pi * x)) .* sin(w*t);

figure(2);
plot(t, azimuth, 'LineWidth', 2);
grid on;

fprintf('AZ Scanning Beam Lookup Table\n');
for k = 1:length(azimuth)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(azimuth((k-1) + m), low1, high1, low2, high2));
    end
    fprintf('\n');
end

fprintf('\n\n');

