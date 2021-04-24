clc;

x0 = [0.5 0.5]; % initial condition
tspan = [0 5]; % time span for simulation

[ta, xa] = ode45(@(t,x)prob(t,x), tspan, x0); % simulate the ODE
cnt = size(xa);
Va = zeros(cnt(1),1);
for i = 1:cnt
    Va(i) = xa(i,1)*xa(i,1) + xa(i,2)*xa(i,2);
end

figure
hold on
plot(ta, xa(:,1), 'LineWidth',2)
plot(ta, xa(:,2), 'LineWidth',2)
plot(ta, Va, 'LineWidth',2)
xlabel('Time (s)')
legend('x1','x2','V')
grid on

% =========================================================================

function xdot = prob(t,x)
% ODE function

xdot = zeros(size(x)) ;
% a
xdot(1) = -2*x(1);
xdot(2) = 2*x(1)-x(2);

% b
%xdot(1) = -x(1)*( 1 + x(2)^2 )*(1 - x(1)^2 - x(2)^2 );
%xdot(2) = -x(2)*( 1 - x(1)^2 )*(1 - x(1)^2 - x(2)^2 );

% c
%xdot(1) = -( x(1)+x(2) ) * ( 1 + 2*(x(1)^2) - 4*(x(1)^4) );
%xdot(2) = x(1) * ( 1 + 2*( x(1)^2) - 4*( x(1)^4) );

% d
%xdot(1) = -x(1) - 5*x(1)*x(2)*x(2);
%xdot(2) = 2*x(1)*x(1)*x(2) - x(2)^3;

end
