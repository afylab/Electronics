% These are the commands you must execute in matlab to intialize
 %global c; c= serial('COM6','baudrate',19200); fopen(c)
 %fwrite(c,[255,254,253,12,55,1,2,3,4,5,6]); % master reset 
 %fwrite(c,[255,254,253,12,7,16,68,0,32,0,0]); % Control register, refclk
% multiplier default is 4 (add to the "68" to increase this

function[] = setacbox_ben(f,x1,y1,x2,y2,theta)
global c % make sure c is initialized already
    refclk=20; %%In MHz
    sysclock=15*refclk*1e6;
%Set frequency, in Hz
    FTW=de2bi(floor((f*2^48)/sysclock),48,'left-msb');
    %disp(ftwB);
    fw1=bi2de(FTW(1:8),'left-msb');
    fw2=bi2de(FTW(9:16),'left-msb');
    fw3=bi2de(FTW(17:24),'left-msb');
    fw4=bi2de(FTW(25:32),'left-msb');
    fw5=bi2de(FTW(33:40),'left-msb');
    fw6=bi2de(FTW(41:48),'left-msb');
    fwrite(c,[255,254,253,12,2,fw1,fw2,fw3,fw4,fw5,fw6]);
     disp('set freq'); disp([255,254,253,12,2,fw1,fw2,fw3,fw4,fw5,fw6]);
%Set phase for DAC 1, in degrees
    PTW=de2bi(floor(mod(theta,360)*2^14/360),16,'left-msb');
    pw1=bi2de(PTW(1:8),'left-msb');
    pw2=bi2de(PTW(9:16),'left-msb');
     disp('set phase'); disp([255,254,253,1,0,pw1,pw2,0,0,0,0])
    fwrite(c,[255,254,253,1,0,pw1,pw2,0,0,0,0]);
%Set amplitude X1
    if x1>1
        x1=1;
    elseif x1<0
        x1=0;
    end
    X1TW=de2bi(floor(x1*(2^12-1)),16,'left-msb');
    x1w1=bi2de(X1TW(1:8),'left-msb');
    x1w2=bi2de(X1TW(9:16),'left-msb');
     disp('set X1'); disp([255,254,253,1,8,x1w1,x1w2,0,0,0,0])
    fwrite(c,[255,254,253,1,8,x1w1,x1w2,0,0,0,0])
%Set amplitude Y1
    if y1>1
        y1=1;
    elseif y1<0
        y1=0;
    end
    Y1TW=de2bi(floor(y1*(2^12-1)),16,'left-msb');
    y1w1=bi2de(Y1TW(1:8),'left-msb');
    y1w2=bi2de(Y1TW(9:16),'left-msb');
     disp('set Y1');  disp([255,254,253,1,9,y1w1,y1w2,0,0,0,0])
    fwrite(c,[255,254,253,1,9,y1w1,y1w2,0,0,0,0])
%Set amplitude X2
    if x1>1
        x1=1;
    elseif x1<0
        x1=0;
    end
    X2TW=de2bi(floor(x2*(2^12-1)),16,'left-msb');
    x2w1=bi2de(X2TW(1:8),'left-msb');
    x2w2=bi2de(X2TW(9:16),'left-msb');
     disp('set X2'); disp([255,254,253,2,8,x2w1,x2w2,0,0,0,0])
    fwrite(c,[255,254,253,2,8,x2w1,x2w2,0,0,0,0])
%Set amplitude Y2
    if y1>1
        y1=1;
    elseif y1<0
        y1=0;
    end
    Y2TW=de2bi(floor(y2*(2^12-1)),16,'left-msb');
    y2w1=bi2de(Y2TW(1:8),'left-msb');
    y2w2=bi2de(Y2TW(9:16),'left-msb');
    disp('set Y2'); disp([255,254,253,2,9,y2w1,y2w2,0,0,0,0])
    fwrite(c,[255,254,253,2,9,y2w1,y2w2,0,0,0,0])
end
