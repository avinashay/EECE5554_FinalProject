% clear; 
close all;
%clc;




 imu10 = readtable('home/avinash/catkin_ws/src/imu/imu.csv');
 mag10 = readtable('home/avinash/catkin_ws/src/imu/magnetometer.csv');
 imu5 = readtable('home/avinash/catkin_ws/src/imu/imu5.csv');
 mag5 = readtable('home/avinash/catkin_ws/src/imu/magnetometer5.csv');

%% allan V
T = 1/40;
allanVariance(imu5(:,19),T,'imu5hour avel x')
allanVariance(imu5(:,20),T,'imu5hour avel y')
allanVariance(imu5(:,21),T,'imu5hour avel z')

allanVariance(imu5(:,31),T,'imu5hour lacc x')
allanVariance(imu5(:,32),T,'imu5hour lacc y')
allanVariance(imu5(:,33),T,'imu5hour lacc z')

%% mean & std

lacc_x_mean = mean(table2array(imu10(:,'linear_acceleration_x')))
lacc_y_mean = mean(table2array(imu10(:,'linear_acceleration_y')))
lacc_z_mean = mean(table2array(imu10(:,'linear_acceleration_z')))
lacc_x_std = std(table2array(imu10(:,'linear_acceleration_x')))
lacc_y_std = std(table2array(imu10(:,'linear_acceleration_y')))
lacc_z_std = std(table2array(imu10(:,'linear_acceleration_z')))

avel_x_mean = mean(table2array(imu10(:,'angular_velocity_x')))
avel_y_mean = mean(table2array(imu10(:,'angular_velocity_y')))
avel_z_mean = mean(table2array(imu10(:,'angular_velocity_z')))
avel_x_std = std(table2array(imu10(:,'angular_velocity_x')))
avel_y_std = std(table2array(imu10(:,'angular_velocity_y')))
avel_z_std = std(table2array(imu10(:,'angular_velocity_z')))

mag_x_mean = mean(table2array(mag10(:,'magnetic_field_x')))
mag_y_mean = mean(table2array(mag10(:,'magnetic_field_y')))
mag_z_mean = mean(table2array(mag10(:,'magnetic_field_z')))
mag_x_std = std(table2array(mag10(:,'magnetic_field_x')))
mag_y_std = std(table2array(mag10(:,'magnetic_field_y')))
mag_z_std = std(table2array(mag10(:,'magnetic_field_z')))


%%
figure
hold on 
plot(table2array(imu10(:,'linear_acceleration_x')))
plot(table2array(imu10(:,'linear_acceleration_y')))
plot(table2array(imu10(:,'linear_acceleration_z')))
legend('lacc-x','lacc-y','lacc-z')
title('linear acceleration xyz')

figure
hold on 
plot(table2array(imu10(:,'angular_velocity_x')))
plot(table2array(imu10(:,'angular_velocity_y')))
plot(table2array(imu10(:,'angular_velocity_z')))
legend('avel-x','avel-y','avel-z')
title('angular velocity xyz')

figure
hold on 
plot(table2array(mag10(:,'magnetic_field_x')))
plot(table2array(mag10(:,'magnetic_field_y')))
plot(table2array(mag10(:,'magnetic_field_z')))
legend('mag-x','mag-y','mag-z')
title('magnetic field xyz')


%% function

function allanVariance(omega,tau0,title_name)
%from mathwork web
theta = cumsum(table2array(omega),1)*tau0;
maxNumM = 100;
L = size(theta,1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*tau0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));

adev = sqrt(avar);

title_name = strcat(title_name,'-Allan Deviation')
figure
% figure
% loglog(tau, adev)
% title(title_name)
% xlabel('\tau');
% ylabel('\sigma(\tau)')
% grid on
% axis equal

%% Angle Random Walk
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);
%figure
subplot(2,2,1)
loglog(tau, adev, tau, lineN, '--', tauN, N, 'o')
%subplot(, 'XScale', 'log', 'YScale', 'log')
title(strcat(title_name,'Allan Deviation with Angle Random Walk'))
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_N')
text(tauN, N, 'N')
grid on
axis equal


%% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);
%figure
subplot(2,2,2)
loglog(tau, adev, tau, lineK, '--', tauK, K, 'o')
title(strcat(title_name,'Allan Deviation with Rate Random Walk'))
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_K')
text(tauK, K, 'K')
grid on
axis equal

%% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));
%figure
subplot(2,2,3)
loglog(tau, adev, tau, lineB, '--', tauB, scfB*B, 'o')
title(strcat(title_name,'Allan Deviation with Bias Instability'))
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_B')
text(tauB, scfB*B, '0.664B')
grid on
axis equal

%% conclusion
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
%figure
subplot(2,2,4)
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title(strcat(title_name,'Allan Deviation with Noise Parameters'))
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

end
