% 
close all; clear all;

% serial object
s = serialport("/dev/ttyUSB0",115200);
% open serial port
fopen(s);

% Plotting data
i = 1;
while(1)
   data(i) = single(str2double(fscanf(s)));
   
   if i <= 150
    plot(data,'Linewidth',1.5);
   else
       plot(data(end-150:end),'Linewidth',1.5);
   end
   title('Real Time Serial Data');
   %pause(0.1);
   i = i+1; 
end