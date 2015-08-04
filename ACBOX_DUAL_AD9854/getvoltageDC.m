% dcbox=digitalio('nidaq','Dev1');% addline(dcbox,0:6,'out',{'SCLK';'SDATA';'AD2';'AD1';'AD0';'WCE1';'WCE0'})
% setVoltage(dcbox,0,0);setVoltage(dcbox,0,1);setVoltage(dcbox,0,2);setVoltage(dcbox,0,3);setVoltage(dcbox,0,4);setVoltage(dcbox,0,5);setVoltage(dcbox,0,6);setVoltage(dcbox,0,7)
% These are the commands you must execute in matlab to intialize
% global a
% a = serial('COM5','baudrate',115200)
% fopen(a)
%
 % this is temporary, eventually implement with master instrument smdata structure
function v = getvoltageDC(channel)
global a
switch channel
    case 1
        n1 = 19+128;
        n2=0;
        m1=1;
        m2=0;
    case 2
        n1 = 18+128;
        n2=0;
        m1=1;
        m2=0;
    case 3
        n1 = 17+128;
        n2=0;
        m1=1;
        m2=0;
    case 4
        n1 = 16+128;
        n2=0;
        m1=1;
        m2=0;
    case 5
        n1 = 0;
        n2=19+128;
        m1=0;
        m2=1;
    case 6
        n1 = 0;
        n2=18+128;
        m1=0;
        m2=1;
    case 7
        n1 = 0;
        n2=17+128;
        m1=0;
        m2=1;
    case 8
        n1 = 0;
        n2=16+128;
        m1=0;
        m2=1;
    otherwise
        disp('INVALID CHANNEL')
end
pause(.02);
fwrite(a,[255,254,253,n1,0,0,n2,0,0]);
pause(.02);
fwrite(a,[255,254,253,n1,0,0,n2,0,0]);
pause(.02);
while a.BytesAvailable
    fscanf(a,'%e'); % clear the buffer
end
fwrite(a,[255,254,253,0,0,0,0,0,0]);
pause(.01);

bdata=zeros([1,6]);
for i=0:5;
  i=i+1;
  r=fscanf(a,'%e');
  bdata(i)=r;
end
bdata2=max(bdata(2)*2^8+bdata(3),bdata(5)*2^8+bdata(6));

if bdata2 < 2^15
    %disp(10*bdata2/(2^15-1))
    %bdata3=sprintf('%20f',10.0*bdata2/(2^15-1));
    bdata3=10.0*bdata2/(2^15-1);
else
    %bdata3=sprintf('%20f',-10.0*(2^16-bdata2)/2^15);
    bdata3=-10.0*(2^16-bdata2)/2^15;
    %disp(-10*(2^16-bdata2)/2^15)
end
v = bdata3;
end