% These are the commands you must execute in matlab to intialize

% global c
% c= serial('COM6','baudrate',19200)
% fopen(c)
% fwrite(c,[255,254,253,99,55,1,2,3,4,5,6]); 
% fwrite(c,[255,254,253,99,7,16,68,0,32,0,0]); % Control register, refclk
% % multiplier default is 4 (add to the "68" to increase this
function[] = resetacbox
global c % make sure c is initialized already
fwrite(c,[255,254,253,12,55,0,0,0,0,0,0])
end
