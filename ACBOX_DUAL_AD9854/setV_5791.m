% dcbox=digitalio('nidaq','Dev1');% addline(dcbox,0:6,'out',{'SCLK';'SDATA';'AD2';'AD1';'AD0';'WCE1';'WCE0'})
% setVoltage(dcbox,0,0);setVoltage(dcbox,0,1);setVoltage(dcbox,0,2);setVoltage(dcbox,0,3);setVoltage(dcbox,0,4);setVoltage(dcbox,0,5);setVoltage(dcbox,0,6);setVoltage(dcbox,0,7)
% These are the commands you must execute in matlab to intialize
% global a
% a = serial('COM5','baudrate',115200)
% fopen(a)
%

function[] = setVoltageard(channel,voltage)
global a
    off=0.000;
    scl=1;
    n1=0; %selects 
    nc=0; %selects board
    if voltage > 10
        voltage = 10.0;
    elseif voltage < -10set
        voltage = -10.0;
 end
 voltage=(voltage-off)/scl;

switch channel
    case 0
        n1=20;
        nc=1;
    case 1
        n1 = 19;
        nc=1;
    case 2
        n1 = 18;
        nc=1;
    case 3
        n1 = 17;
        nc=1;
    case 4
        n1 = 16;
        nc=1;
    case 5
        n1 = 19;
        nc=2;
    case 6
        n1 = 18;
        nc=2;
    case 7
        n1 = 17;
        nc=2;
    case 8
        n1 = 16;
        nc=2;
    otherwise
       disp('INVALID CHANNEL')
end
  if voltage >= 0
        temp = round((2^15-1)*voltage/10);
    else
        temp = round(2^16 - abs(voltage)/10 * 2^15);    
  end
 
disp('16bit=')
disp(temp)

v16 = de2bi(temp,16,2,'left-msb');
%disp('16bitbin=')
%disp(v16)
%disp(bi2de(fliplr(v16(1:8))))
%disp(bi2de(fliplr(v16(9:16))))
n2=bi2de(fliplr(v16(1:8)));
n3=bi2de(fliplr(v16(9:16)));
 disp(nc)
 disp(n1)
 disp(n2)
 disp(n3)
fwrite(a,[nc,n1,n2,n3])
end
