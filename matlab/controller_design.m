%% LQR controller
l = 0.1;
Iyy = 1.436e-5;
Ixx = 1.395e-5;
Izz = 2.173e-5;
g = 9.81;
m = 0.027;
k = 2.75e-11;
b = 1e-9;

A = zeros(8);
B = zeros(8,4);

A(1,2) = 1;
A(3,4) = 1;
A(5,6) = 1;
A(7,8) = 1;

B(2,:) = -ones(4,1)/m;
B(4,:) = [-l -l l l] /(Ixx * sqrt(2)); % theta roll
B(6,:) = [l -l -l l] /(Iyy * sqrt(2)); % phi pitch
B(8,:) = [l -l l -l] * k/(b*Izz); % psi yaw
%B(4,1) = l/Iyy; B(6,3) = -l/Iyy;
%B(6,:) = alpha/Izz *[1 -1 1 -1];
C = eye(size(A));
Q = diag([1000 100 10000 100 10000 100 0.001 10 ]); % punish angular not angular velocity
R = diag([1 1 1 1]);

[K, ~, E] = lqr(A, B, Q, R)
%% pwm
clear all
% MAX thrust (data from https://www.bitcraze.io/crazyflie-2/)
g = 9.81; %m/s^2
m = 0.027; % kg
take_off_weight = 0.042; % g
TMaxGrams = (m + take_off_weight)*1e3; % grams
TMaxNewton = TMaxGrams * g*1e-3
mg = m*g
%% PWM mapping
%(data from https://wiki.bitcraze.io/misc:investigations:thrust)

% thrust in grams converted to Newton and divided by the number of motors
x = 9.81e-3/4 * [0.0 1.6 4.8 7.9 10.9 13.9 17.3 21.0 24.4 28.6 32.8 37.3 41.7 46.0 51.9 57.9];
% pwm in % converted to u_int16
y = 2^16/100 *[0 6.25 12.5 18.75 25 31.25 37.5 43.25 50 56.25 62.5 68.75 75 81.25 87.5 93.75];
A = polyfit(x,y,2)
A2 = polyfit(x,y,1)
TtoPWM= @(T) A(1)*T.^2 + A(2)*T + A(3);
TtoPWM2= @(T) A(1)*T.^2 + A(2)*T;
TtoPWM3= @(T) A2(1)*T + A2(2);
TtoPWM(0.6)
TtoPWM(0)


plot(x, y, '-o')
hold on
plot(x, TtoPWM(x))
plot(x, TtoPWM2(x))
%plot(x, TtoPWM3(x))
hold off

ylabel('PWM (0-2^{16} bits)')
xlabel('Thrust (N)')
legend('table data', 'T to PWM from table data', 'T to PWM w/o c')