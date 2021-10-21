clear all
close all

% data = table2array(readtable("data/datafile_line.csv"));
data = table2array(readtable("data/datafile_scurve.csv"));
leftMotor = data(:,1);
rightMotor = data(:,2);
leftSensor = data(:,3);
rightSensor = data(:,4);

figure()
hold on
yyaxis left
plot(leftMotor,'b-')
plot(rightMotor,'r-')
ylabel("Motor Speed")

yyaxis right
plot(leftSensor,'c-')
plot(rightSensor,'m-')
ylabel("Sensor Value")

legend("Left Motor","Right Motor","Left Sensor","Right Sensor")
xlabel("Time (steps)")
% title("Motor and Sensor Values, Straight Line Test")
title("Motor and Sensor Values, S-Curve Test")
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure()
hold on
subplot(1,2,1)
yyaxis left
plot(leftMotor,'b-')
% plot(rightMotor,'r-')
ylabel("Motor Speed")

yyaxis right
plot(leftSensor,'r-')
% plot(rightSensor,'m-')
ylabel("Sensor Value")

legend("Left Motor","Left Sensor")
xlabel("Time (steps)")
% title("Left Motor and Sensor, Straight Line Test")
title("Left Motor and Sensor, S-Curve Test")

subplot(1,2,2)
yyaxis left
% plot(leftMotor,'b-')
plot(rightMotor,'b-')
ylabel("Motor Speed")

yyaxis right
% plot(leftSensor,'c-')
plot(rightSensor,'r-')
ylabel("Sensor Value")

legend("Right Motor","Right Sensor")
xlabel("Time (steps)")
% title("Right Motor and Sensor, Straight Line Test")
title("Right Motor and Sensor, S-Curve Test")
