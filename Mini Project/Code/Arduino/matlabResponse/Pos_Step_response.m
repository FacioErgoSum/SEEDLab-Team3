%Position Step Response
clear all
close all
clc

port = "COM3";
baud = 115200;
s = serialport(port, baud);

configureTerminator(s,"CR/LF");

flush(s);

itt = 1;
pos = zeros(1,10000);
while itt <= 10000
    pos(itt) = readline(s);
    itt = itt + 1;
end
x = linspace(0,10,10000);
configureCallback(s, "off");
 plot(x,pos);
 K = 12.5;
 sigma = .6;
open_system("The_BIG_SAD.slx");
out = sim("The_BIG_SAD.slx");
close_system("The_BIG_SAD");
plot(out.simout);
