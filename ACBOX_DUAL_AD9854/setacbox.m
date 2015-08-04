% These are the commands you must execute in matlab to intialize

% global c
% c= serial('COM6','baudrate',19200)
% fopen(c)
% fwrite(c,[255,254,253,12,55,1,2,3,4,5,6]); % master reset 
% fwrite(c,[255,254,253,12,7,16,68,0,32,0,0]); % Control register, refclk
% % multiplier default is 4 (add to the "68" to increase this
function[] = setacbox(f,x1,y1,x2,y2,theta)
global c % make sure c is initialized already
    refclk=20; %%In MHz
    sysclock=10*refclk*1e6;
%Set frequency, in Hz
    FTW=dec2bin(floor((f*2^48)/sysclock),48);
    %disp(ftwB);
    fw1=bin2dec(FTW(1:8));fw2=bin2dec(FTW(9:16));fw3=bin2dec(FTW(17:24));fw4=bin2dec(FTW(25:32));fw5=bin2dec(FTW(33:40));fw6=bin2dec(FTW(41:48));
    fwrite(c,[255,254,253,12,2,fw1,fw2,fw3,fw4,fw5,fw6]);
    % disp('set freq');
    %disp([255,254,253,12,5x2,fw1,fw2,fw3,fw4,fw5,fw6]);
%Set phase for DAC 1, in degrees
    PTW=dec2bin(floor(mod(theta,360)*2^14/360),16);
    pw1=bin2dec(PTW(1:8));
    pw2=bin2dec(PTW(9:16));
    % disp('set phase')
    % disp([255,254,253,1,0,pw1,pw2,0,0,0,0])
    fwrite(c,[255,254,253,1,0,pw1,pw2,0,0,0,0]);
    fwrite(c,[255,254,253,2,0,0,0,0,0,0,0]);

%Set amplitude X1
    if x1>1
        x1=1;
    elseif x1<0
        x1=0;
    end
    X1TW=dec2bin(floor(x1*(2^12-1)),16);
    x1w1=bin2dec(X1TW(1:8));
    x1w2=bin2dec(X1TW(9:16));
    % disp('set X1');
  %   disp([255,254,253,1,8,x1w1,x1w2,0,0,0,0])
    fwrite(c,[255,254,253,1,8,x1w1,x1w2,0,0,0,0])
%Set amplitude Y1
    if y1>1
        y1=1;
    elseif y1<0
        y1=0;
    end
    Y1TW=dec2bin(floor(y1*(2^12-1)),16);
    y1w1=bin2dec(Y1TW(1:8));
    y1w2=bin2dec(Y1TW(9:16));
    % disp('set Y1');
   %  disp([255,254,253,1,9,y1w1,y1w2,0,0,0,0])
    fwrite(c,[255,254,253,1,9,y1w1,y1w2,0,0,0,0])
%Set amplitude X2
    if x1>1
        x1=1;
    elseif x1<0
        x1=0;
    end
    X2TW=dec2bin(floor(x2*(2^12-1)),16);
    x2w1=bin2dec(X2TW(1:8));
    x2w2=bin2dec(X2TW(9:16));
    % disp('set X2');
   %  disp([255,254,253,2,8,x2w1,x2w2,0,0,0,0])
    fwrite(c,[255,254,253,2,8,x2w1,x2w2,0,0,0,0])
%Set amplitude Y2
    if y1>1
        y1=1;
    elseif y1<0
        y1=0;
    end
    Y2TW=dec2bin(floor(y2*(2^12-1)),16);
    y2w1=bin2dec(Y2TW(1:8));
    y2w2=bin2dec(Y2TW(9:16));
    % disp('set Y2');
    % disp([255,254,253,2,9,y2w1,y2w2,0,0,0,0])
    fwrite(c,[255,254,253,2,9,y2w1,y2w2,0,0,0,0])
%Update synchronized DACs
fwrite(c,[255,254,253,1,101,0,0,0,0,0,0])
end
