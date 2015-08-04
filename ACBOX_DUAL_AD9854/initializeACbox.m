% initialization AC box

global c; 
% c = serial('COM4','baudrate',19200); 
%fofcpen(c);
%%
mr = [255,254,253,12,55,0,0,0,0,0,0];
cr = [255,254,253,12,7,16,64+5,0,32,0,0]; %[255,254,253,12,7,16,68,0,32,0,0]
fwrite(c,mr);
pause(.5)
fwrite(c,cr);
%%
%setacbox(1234,1,1,1,1,0)

%fclose(cfop);