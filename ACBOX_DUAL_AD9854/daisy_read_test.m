global c
del=0.005;
tic
while c.BytesAvailable
      fscanf(c,'%s');
 end
% bytes = c.BytesAvailable
fwrite(c,[255,254,253,20,64,0,20,230,102])
pause(del)
% bytes = c.BytesAvailable

 while c.BytesAvailable
      fscanf(c,'%s');
 end
% bytes = c.BytesAvailable
pause(del)
fwrite(c,[255,254,253,144,0,0,144,0,0]);
pause(del)
% bytes = c.BytesAvailable 
 
 while c.BytesAvailable
     fscanf(c,'%s');
 end
 
 pause(del)
%  bytes = c.BytesAvailable
 fwrite(c,[255,254,253,0,0,0,0,0,0]);
 pause(del)
%  bytes = c.BytesAvailable
clearvars q
for i=0:5;
  i=i+1;
  r=fscanf(c,'%e');
  q(i)=r;
end
if (q(2)*2^8+q(3)) < 2^15
    disp(10*(q(2)*2^8+q(3))/(2^15-1))
else
    disp(-10*(2^16-(q(2)*2^8+q(3)))/2^15)
end

if (q(5)*2^8+q(6)) < 2^15
    disp(10*(q(5)*2^8+q(6))/(2^15-1))
else
    disp('neg')
    disp(-10*(2^16-(q(5)*2^8+q(6)))/2^15)
end
toc
