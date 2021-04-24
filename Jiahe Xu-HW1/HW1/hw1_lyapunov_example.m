% ME 530.678 Nonlinear Control
% Homework 1, Problem 2 example for scalar system

x0 = 0.5; % initial condition
tspan = [0 5]; % time span for simulation

[ta, xa] = ode45(@(t,x)prob(t,x), tspan, x0); % simulate the ODE
Va = 0.5*xa.^2; % calculate Lyanpunov function over time span

figure
hold on
plot(ta, xa, 'LineWidth',2)
plot(ta, Va, 'LineWidth',2)
xlabel('Time (s)')
legend('x','V')
grid on
title('Example')

% =========================================================================

function xdot = prob(t,x)
% ODE function

xdot = -x;

end
