% dcbox=digitalio('nidaq','Dev1');% addline(dcbox,0:6,'out',{'SCLK';'SDATA';'AD2';'AD1';'AD0';'WCE1';'WCE0'})
% setVoltage(dcbox,0,0);setVoltage(dcbox,0,1);setVoltage(dcbox,0,2);setVoltage(dcbox,0,3);setVoltage(dcbox,0,4);setVoltage(dcbox,0,5);setVoltage(dcbox,0,6);setVoltage(dcbox,0,7)
% These are the commands you must execute in matlab to intialize
% global a
% a = serial('COM5','baudrate',115200)
% fopen(a)
%

function[] = setV5791(voltage)
global a
    off=0.000;
    scl=1.0;
    n1=0; %selects 
    nc=0; %selects board
    if voltage > 10
        voltage = 10.0;
    elseif voltage < -10
        voltage = -10.0;
 end
 voltage=(voltage-off)/scl;

% switch channel
%     case 0
%         n1=20;
%         nc=1;
%     case 1
%         n1 = 19;
%         nc=1;
%     case 2
%         n1 = 18;
%         nc=1;
%     case 3
%         n1 = 17;
%         nc=1;
%     case 4
%         n1 = 16;
%         nc=1;
%     case 5
%         n1 = 19;
%         nc=2;
%     case 6
%         n1 = 18;
%         nc=2;
%     case 7
%         n1 = 17;
%         nc=2;
%     case 8
%         n1 = 16;
%         nc=2;
%     otherwise
%        disp('INVALID CHANNEL')
% end
  if voltage >= 0
        temp = round((2^19-1)*voltage/10);
    else
        temp = round(2^20 - abs(voltage)/10 * 2^19);    
  end
 
disp('20 bit=')
disp(temp)

v20 = de2bi(temp,20,2,'left-msb');
reg4=[0 0 0 1];
disp('20bitbin=')
disp(v20)
%  disp(bi2de(fliplr([reg4, v20(1:4)])))
%  disp(bi2de(fliplr(v20(5:12))))
%  disp(bi2de(fliplr(v20(13:20))))
n1=bi2de(fliplr([reg4, v20(1:4)]));
n2=bi2de(fliplr(v20(5:12)));
n3=bi2de(fliplr(v20(13:20)));
 disp(n1)
 disp(n2)
 disp(n3)
fwrite(a,[n1,n2,n3])
end
