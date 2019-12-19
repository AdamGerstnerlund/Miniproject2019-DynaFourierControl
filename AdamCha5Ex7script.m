%% Script for exercise 7 from Lecture Notes for Data Acquisition chapter 5.
% A small MATLAB prg. identifying the Fourier series ftc. y(t) = (-2*t^2)
% in the open interval (-pi, pi) or -pi < t < pi.
% Script should run smoothly in ML without additional toolboxes.

clear
clc
close all

% Define Symbolics t, n:
syms t
syms n

% Constant a_zero:
Fa_0 = @(t) -2*t.^2;
Qa_0 = integral(Fa_0, -pi, pi);

% Time vars.:
T = 2*pi; % Second. Periodic
N = 150;  % Resolution
i = 0;    % int constant, starting at zero, incremented by one (loop)

% Loop computing the Fourier series. Coefficients a_n, b_n were found by hand:
for t = linspace(-T,T,N)
    i = i + 1; time(i) = t;
    y(i) = -2*t^2;
    a_0 = (1)/(2*pi)*(Qa_0);
    a_n = (-1)/(pi*(n.^3))*( 4*((pi.^2)*sin(n*pi)*n.^2 + 2*n*cos(n*pi)*pi - 2*sin(n*pi)) );
    b_n = 0;
    sum_1 = ( a_n*cos(n*t) + b_n*sin(n*t) );
    f(i) = a_0 + symsum(sum_1, n, 1, 10); % N=10
    x(i) = a_0 + symsum(sum_1, n, 1, 1); % N=1
end

% To plot the graph and legends in a monitor window:
figure(1)
clf
figure(1)
hold on
plot(time, y, 'b', 'Linewidth', 1)
plot(time, x, 'r', 'Linewidth', 1)
plot(time, f, 'g', 'Linewidth', 1)
hold off
title('Fourier series plot')
legend('Given ftc.', 'N = 1', 'N = 10')
grid on;
xlabel('Time [s]');
xlim([-6.4 6.4]);
ylim([-25 5]);
