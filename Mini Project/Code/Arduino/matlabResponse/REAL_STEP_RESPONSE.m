%% STEP RESPONSE
clear all
close all
clc
%configure the configuration on the serial port
port = "COM3"; %Check on the system under devices
baud = 115200;
%Create a serial communication object
arduino = serialport(port, baud);

%configure the terminotaor with the Carrige Return and Line Feed values
configureTerminator(arduino, "CR/LF"); %Value of 13,10; respectivly

%clear the chanel of any remaing values
flush(arduino);

arduino.UserData = struct("AngVel", [], "Time", 1);
angVel = 0;
itt = 1;
%configureCallback(arduino,"terminator", @pollDUT);

while itt <= 10000
    angVel(itt) = readline(arduino);
    itt = itt + 1;
end

configureCallback(arduino,"off");

x = linspace(0,10,10000);

plot(x, angVel);
%%

 K = 12.34;
 sigma = .65;
% 
open_system("Motor_Model.slx");
out = sim("Motor_Model.slx");
close_system("Motor_Model.slx");
plot(out.simout)


