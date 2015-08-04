function responseMatrix = Capbridge(varargin)
global smdata

% Default values
liTmp = smchaninst('liStatus');
liInd = liTmp(1);
gainMargin   = 1.2;
acFrequency  = 100000; % 10 kHz
tc          = .05;
measurements = 1; % Number of measurements to take
% DCinput      = true;
% groundInput  = true;
refattendB=0;
sigattendB=0;
refatten     = 10^(-(refattendB) / 20);
sigatten     = 10^(-(sigattendB) / 20);
% relayDelay   = 0.01;
staticStepRange = 0.25; % How much to vary the standard signal when
                       % calculating the initial zero point.
initialExcitation = .75; % [Real imaginary]
initialReference  = .9; % [Real imaginary]

% If we know something about the system, these values should be non-zero.
% Otherwise leave them as is and let the program try to find the balance
% point on its own.
X0 = 0;
Y0 = 0;
dX = 0;
dY = 0;
minx = -1.0;
miny = -1.0;
maxx = 1.0;
maxy = 1.0;

%tmp = smget('liTC'); % If a function returns a cell, you can't
% get the value from the cell without storing it in a temporary variable
% first.
%tc = tmp{1};
tau = tc * 3.0;
avg = 0.5;

whileCounter = 0;
% Initialization
smset('mreset',0);
smset('sendctrlregister',0);
smget('liWait'); % Setup lockin if hasn't been already
smset('acbox_freq',acFrequency);
smset('acbox_ch0x',initialReference);
smset('acbox_ch0y',initialExcitation);
smset('acbox_ch1x',initialReference);
smset('acbox_ch1y',initialExcitation);
smset('acbox_phase',180);

% Choose a sensible initial scale based on bridge balancing
initScale=initialExcitation*sigatten*.125;
disp(initScale)
smset('liScale',initScale/2);
smset('liReserve',[5 0]) % 5 is reserveMagic. Sorry for the hard coding.
smset('liTC',tc);

smget(

% 
% while 1
%     whileCounter = whileCounter + 1;
%     if whileCounter > 10
%         cprintf('err','ERROR: Capbridge unable to get good reading. Exiting\n');
%         return
%     end
%     amp=sqrt((X0+minx*staticStepRange)^2+(Y0+miny*staticStepRange)^2);
%     ph=180*atan((Y0+miny*staticStepRange)/(X0+minx*staticStepRange))/pi;
%     smset('acbox_ch1x',amp);
%     tmp1 = smget('acbox_ch1x');
%     amp=tmp1{1};
%     smset('acbox_phase',ph);
%     tmp2 = smget('acbox_phase');
%     ph=tmp2{1};
%     X0 =amp*cos(ph);
%     Y0 =amp*sin(ph);
%     source1(1,1) = X0;
%     source1(1,2) = Y0;
%     pause(tau);
%      
%     fprintf('\nCorner 1\n\n');
%     % Have to use the driver directly instead of smget since smget doesn't
%     % accept arguments and smset doesn't return a value.
%     liInd = smchaninst('liTC');
%     liInd = liInd(1);
%      
%      if strcmp(smdata.inst(liInd).device,'SR7280')
%          rVal = smcSR7280([smchaninst('liGetAve') 0],[1 0.4 true]);
%          disp('7280 7280 7280 7280 ')
%      elseif strcmp(smdata.inst(liInd).device,'SR830ashoori')
%          rVal = smcSR830ashoori([smchaninst('liGetAve') 0],[1 0.4 true]);
%      else
%          cprintf('err','ERROR: LOCK-IN TYPE NOT KNOWN BY CAPBRIDGE, EDIT CAPBRIDGE TO ACCOMODATE NEW LOCK-IN\N');
%      end
%     
%     if rVal{1} < 0
%         cprintf('red','Warning: Lockin overload. Lowering scale and restarting\c');
%         if bitand(smdata.inst(liInd).data.accumStatus,smdata.inst(liInd).data.statusInputOver)
%             smset('liUpRes',0);
%         else
%             smset('liDownScale',0);
%         end
%         pause(tau);
%         continue;
%     end
%     meas1(1,1)  = rVal{2};
%     meas1(1,2)  = rVal{3};
%     measS1(1,1) = rVal{4};
%     measS1(1,2) = rVal{5};
%     
%     
%     smset('acstd',[X0+maxx*staticStepRange Y0+maxy*staticStepRange]);
%     tmp = smget('acstd');
%     X1 = tmp{1}(1);
%     Y1 = tmp{1}(2);
%     source2(1,1) = X1;
%     source2(1,2) = Y1;
%     pause(tau);
%     
%     fprintf('\nCorner 2\n\n');
%     
%     if strcmp(smdata.inst(liInd).device,'SR7280')
%         rVal = smcSR7280([smchaninst('liGetAve') 0],[1 0.4 true]);
%     elseif strcmp(smdata.inst(liInd).device,'SR830ashoori')
%         rVal = smcSR830ashoori([smchaninst('liGetAve') 0],[1 0.4 true]);
%     else
%         cprintf('err','ERROR: LOCK-IN TYPE NOT KNOWN BY CAPBRIDGE, EDIT CAPBRIDGE TO ACCOMODATE NEW LOCK-IN\N');
%     end
%     
%     if rVal{1} < 0
%         cprintf('red','Warning: Lockin overload. Lowering scale and restarting\c');
%         if bitand(smdata.inst(liInd).data.accumStatus,smdata.inst(liInd).data.statusInputOver)
%             smset('liUpRes',0);
%         else
%             smset('liDownScale',0);
%         end
%         pause(2*tau);
%         continue;
%     end
%     meas2(1,1)  = rVal{2};
%     meas2(1,2)  = rVal{3};
%     measS2(1,1) = rVal{4};
%     measS2(1,2) = rVal{5};
%     
%     fprintf('\nCapbridge measurements completed. Calculating balance point\n\n');
%     break
% end
    
%     dVs = source2 - source1;
%     dVm = meas2 - meas1;
%     disp('This was just changed')
%     theta = atan2(dVm(2),dVm(1)) - atan2(dVs(2),dVs(1));
%     r = norm(dVm)/norm(dVs);
%     responseMatrix = r*[cosd(theta) -sind(theta); sind(theta) cosd(theta)];
%     offset = meas1 - (responseMatrix*source1')';
%     
%     target = (-inv(responseMatrix)*offset')';
%     smset('acstd',target);
%     fprintf('Standard capacitance X: %f, Y: %f\n',target(1)/initialExcitation(1),target(2)/initialExcitation(1));
end