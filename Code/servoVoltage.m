clc; clear; close all;

% The following script is to fit a best line on the data points for the 
% duty cycle we are setting and the voltage we are getting from 
% potentiometer and mapping that to the angles made my the servos

% servo A
% x = [ ...
%     3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, ...
%     8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12 ];
% 
% y = [ 0.42, 0.51, 0.59, 0.67, 0.76, 0.84, 0.92, 1.01, 1.09, 1.17, ...
%     1.25, 1.34, 1.42, 1.51, 1.59, 1.67, 1.75, 1.83, 1.91 ];

% servo B
x = [ 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, ...
    8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12, 12.5, 12.7 ];

y = [ 0.49, 0.57, 0.65, 0.73, 0.82, 0.90, 0.98, 1.06, 1.14, 1.22, ...
    1.30, 1.38, 1.46, 1.54, 1.62, 1.7, 1.78, 1.85, 1.93, 1.96 ];

% servo C
% x = [ 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, ...
%     8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12 ];
% 
% y = [ 0.41, 0.5, 0.58, 0.66, 0.74, 0.83, 0.91, 0.99, 1.07, 1.15, ...
%     1.23, 1.32, 1.4, 1.48, 1.56, 1.64, 1.72, 1.8, 1.88 ];
 
% Linear best-fit (y = m*x + c)
p = polyfit(x, y, 1);   % 1st order polynomial
m = p(1);
c = p(2);

fprintf('Best fit line:\n');
fprintf('y = %.4f*x + %.4f\n', m, c);

% Generate fitted values
x_fit = linspace(min(x), max(x), 100);
y_fit = polyval(p, x_fit);

% Plot
figure;
plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5); % data points
hold on;
plot(x_fit, y_fit, 'b-', 'LineWidth', 2);           % best-fit line
grid on;

xlabel('Duty Cycles (%)');
ylabel('Potentiometer Output (V)');
title('Best-fit for Servo B');
legend('Data Points', 'Best-Fit Line', 'Location', 'northwest');