
close all;
clear; 
clc;

% Generate Lookup Tables

low1 = -1;
high1 = 1;
low2 = 0;
high2 = 4095;

AZ_lim = [-62, 62];
EL_lim = [-1.5, 29.5];
BAZ_lim = [-42, 42];
HRAZ_lim = [-42, 42];


f = 78.125e3; % Given carrier frequency
w = 2 * pi * f;
dac = @(value, input_low, input_high, output_low, output_high) round( output_low + (value - input_low) * (output_high - output_low) / (input_high - input_low) );

line_length = 10;

sample_num = 100;


dfiles = ['AZ_Table', 'EL_Table', 'BAZ_Table'];
for k = 1:length(dfiles)
    if exist(dfiles(k), 'file') ; delete(dfiles(k)); end
end

%% Standard Sine Wave at Carrier Frequency

t = linspace(0, 1/f, sample_num);

%dt = 0.01;
%clk_cycles = 124e2;
%t = [0:dt:clk_cycles*dt];

carrier_wave = 0.1*0.125*sin(w*t);

figure(1);
plot(t, carrier_wave, 'LineWidth', 2);
grid on;

fprintf('Carrier Signal Lookup Table\n');
for k = 1:sample_num/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(carrier_wave(line_length*(k - 1) + m ), low1, high1, low2, high2) );
    end
    fprintf('\n');
end

fprintf('\n\n');

%% Azimuth Sinc Pulse

diary AZ_Table

ang_min = AZ_lim(1);
ang_max = AZ_lim(2);
bw = 2;
a = 10;

%t = linspace(0, 50*(ang_max - ang_min), sample_num);
%t = linspace(0, 50*(ang_max - ang_min), 10*sample_num);
dt = 0.01;
clk_cycles = 124e1;
t = [0:dt:clk_cycles*dt];


inc = (dt*clk_cycles) / (ang_max - ang_min);

theta_t = ang_min + (t ./ inc);
theta_r = -10;

x = (theta_t - theta_r) / (1.15 * bw);

carrier = sin(w*t);

azimuth = a * (sinc(x)) .* carrier;

figure(2);
plot(t, azimuth, 'LineWidth', 2);
title('Azimuth');
xlabel('t');
ylabel('Voltage');
grid on;

fprintf('AZ Scanning Beam Lookup Table\n');
for k = 1:length(azimuth)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(azimuth(line_length*(k-1) + m), a*low1, a*high1, low2, high2));
    end
    fprintf('\n');
end

fprintf('\n\n');

diary off
%% Elevation Sinc Pulse

diary EL_Table

ang_min = EL_lim(1);
ang_max = EL_lim(2);
bw = 1.5;
a = 10;

%t = linspace(0, 50*(ang_max - ang_min), sample_num);
%t = linspace(0, 50*(ang_max - ang_min), 10*sample_num);
dt = 0.01;
clk_cycles = 31e1;
t = [0:dt:clk_cycles*dt];

inc = (dt*clk_cycles) / (ang_max - ang_min);

theta_t = ang_min + (t ./ inc);
theta_r = 3;

x = (theta_t - theta_r) / (1.15 * bw);

carrier = sin(w*t);

elevation = a * (sinc(x)) .* carrier;

figure(3);
plot(t, elevation, 'LineWidth', 2);
title('Elevation');
xlabel('t');
ylabel('Voltage');
grid on;

fprintf('EL Scanning Beam Lookup Table\n');
for k = 1:length(elevation)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(elevation(line_length*(k-1) + m), a*low1, a*high1, low2, high2));
    end
    fprintf('\n');
end

fprintf('\n\n');

diary off
%% Back Azimuth Sinc Pulse

diary BAZ_Table

ang_min = BAZ_lim(1);
ang_max = BAZ_lim(2);
bw = 2;
a = 10;

%t = linspace(0, 50*(ang_max - ang_min), sample_num);
%t = linspace(0, 50*(ang_max - ang_min), 10*sample_num);

dt = 0.01;
clk_cycles = 84e1;
t = [0:dt:clk_cycles*dt];

inc = (dt*clk_cycles) / (ang_max - ang_min);

theta_t = ang_min + (t ./ inc);
theta_r = -10;

x = (theta_t - theta_r) / (1.15 * bw);

carrier = sin(w*t);

backazimuth = a * (sinc(x)) .* carrier;

figure(4);
plot(t, backazimuth, 'LineWidth', 2);
title('Back-Azimuth');
xlabel('t');
ylabel('Voltage');
grid on;

fprintf('BAZ Scanning Beam Lookup Table\n');
for k = 1:length(backazimuth)/line_length
    for m = 1:line_length
        fprintf('%d, ', dac(backazimuth(line_length*(k-1) + m), a*low1, a*high1, low2, high2));
    end
    fprintf('\n');
end

diary off