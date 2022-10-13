

% serialportlist % to list

port =  "/dev/ttyACM1";
baudrate = 115200 ;

% s = serialport(port,baudrate); % to connect
configureTerminator(s,"CR/LF");

% read(s)	%Read data from serial port
% readline(s)	%Read line of ASCII string data from serial port
% write	%Write data to serial port
% writeline	%Write line of ASCII data to serial port
y = "";
%format long
x = [0 , 10  , 0 , 0,  0  ,0];
x = round(x*100)/100 ;
for i = 1 : length(x)
    y = y + x(i);
    if i ~= length(x)
        y = y + ',';
    end
end
y = '111.000;10.000,11.000,22.000,33.000,44.000';
y2 = unicode2native(y, 'utf-8') ;
y2(43) = 10;
y3 = typecast(x, 'uint8');
% y = bytes(y)
% y = 0
% text = ('%.3f | %.3f | %.3f', y)

while(1)
   write(s,y2,'int8');
%     writeline(s,x);
    pause(0.05);
    data = readline(s)
%     read(device,5,"uint8")

end
    
   
clear s