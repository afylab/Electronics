% dcbox=digitalio('nidaq','Dev1');% addline(dcbox,0:6,'out',{'SCLK';'SDATA';'AD2';'AD1';'AD0';'WCE1';'WCE0'})
% setVoltage(dcbox,0,0);setVoltage(dcbox,0,1);setVoltage(dcbox,0,2);setVoltage(dcbox,0,3);setVoltage(dcbox,0,4);setVoltage(dcbox,0,5);setVoltage(dcbox,0,6);setVoltage(dcbox,0,7)
% These are the commands you must execute in matlab to intialize
% global a
% a = serial('COM5','baudrate',115200)
% fopen(a)
%

%Mac:  global a; a = serial('/dev/tty.usbmodem1431','BaudRate',115200); fopen(a)


 % this is temporary, eventually implement with master instrument data structure
function[] = calibrateSeekat(channel)

global a

switch channel
    case 1
        n1 = 19;
        n2=0;
        m1=1;
        m2=0;
    case 2
        n1 = 18;
        n2=0;
        m1=1;
        m2=0;
    case 3
        n1 = 17;
        n2=0;
        m1=1;
        m2=0;
    case 4
        n1 = 16;
        n2=0;
        m1=1;
        m2=0;
    case 5
        n1 = 0;
        n2=19;
        m1=0;
        m2=1;
    case 6
        n1 = 0;
        n2=18;
        m1=0;
        m2=1;
    case 7
        n1 = 0;
        n2=17;
        m1=0;
        m2=1;
    case 8
        n1 = 0;
        n2=16;
        m1=0;
        m2=1;
    otherwise
        disp('INVALID CHANNEL')
end

fwrite(a,[255,254,253,n1+24,0,0,n2+24,0,0]); %zero the offset register
fwrite(a,[255,254,253,n1+16,0,0,n2+16,0,0]); %zero the gain register
measuredValues = zeros(2,1);

    voltage = 0;
    if voltage >= 0
        dec16 = round((2^15-1)*voltage/10); %Decimal equivalent of 16 bit data 
    else
        dec16 = round(2^16 - abs(voltage)/10 * 2^15); %Decimal equivalent of 16 bit data
    end

bin16 = de2bi(dec16,16,2,'left-msb'); %16 bit binary
d1=bi2de(fliplr(bin16(1:8))); %first 8 bits
d2=bi2de(fliplr(bin16(9:16))); %second 8 bits
%disp([255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2]);
pause(.005);
fwrite(a,[255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2]);
while a.BytesAvailable
    fscanf(a,'%e');
end
pause(2)

    blah = smget('V');
    offset = -blah{1};
    offsetsteps = round(offset/(38.14e-6));
    offset8 =  de2bi(mod((offsetsteps),2^8),8,'left-msb');
    
    d1=0;%bi2de(fliplr(bin16(1:8))); %first 8 bits
    d2=bi2de(fliplr(offset8));%bi2de(fliplr(bin16(9:16))); %second 8 bits
 
    
    %disp([255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2])
    pause(.005);
    fwrite(a,[255,254,253,n1+24,d1*m1,d2*m1,n2+24,d1*m2,d2*m2]); % +24 to access offset register
    while a.BytesAvailable
        fscanf(a,'%e');
    end
    pause(1)
    
    
    
    voltage = -10;
    if voltage >= 0
        dec16 = round((2^15-1)*voltage/10); %Decimal equivalent of 16 bit data 
    else
        dec16 = round(2^16 - abs(voltage)/10 * 2^15); %Decimal equivalent of 16 bit data
    end

bin16 = de2bi(dec16,16,2,'left-msb'); %16 bit binary
d1=bi2de(fliplr(bin16(1:8))); %first 8 bits
d2=bi2de(fliplr(bin16(9:16))); %second 8 bits
%disp([255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2]);
pause(.005);
fwrite(a,[255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2]);
while a.BytesAvailable
    fscanf(a,'%e');
end
pause(2)

    blah = smget('V');
    gainerror = blah{1}+10;
    gainsteps = round(gainerror/(152.59e-6));
    
    gain8 =  de2bi(mod((gainsteps),2^8),8,'left-msb');
    
    %bin16 = de2bi(gain16,16,2,'left-msb'); %16 bit binary
    d1=0;%bi2de(fliplr(gain16(1:8))) %first 8 bits
    d2=bi2de(fliplr(gain8));%bi2de(fliplr(gain8(1:8))) %second 8 bits
    
    pause(.005);
    fwrite(a,[255,254,253,n1+16,d1*m1,d2*m1,n2+16,d1*m2,d2*m2]); % +16 to access gain register instead of data register
    while a.BytesAvailable
        fscanf(a,'%e');
    end

    pause(1)
    
        voltage = 0;
    if voltage >= 0
        dec16 = round((2^15-1)*voltage/10); %Decimal equivalent of 16 bit data 
    else
        dec16 = round(2^16 - abs(voltage)/10 * 2^15); %Decimal equivalent of 16 bit data
    end

bin16 = de2bi(dec16,16,2,'left-msb'); %16 bit binary
d1=bi2de(fliplr(bin16(1:8))); %first 8 bits
d2=bi2de(fliplr(bin16(9:16))); %second 8 bits
%disp([255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2]);
pause(.005);
fwrite(a,[255,254,253,n1,d1*m1,d2*m1,n2,d1*m2,d2*m2]);
while a.BytesAvailable
    fscanf(a,'%e');
end
    
end