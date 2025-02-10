clc;
clear all;

c= physconst("LightSpeed");

fc = 77e9;
lambda = freq2wavelen(fc,c);

% expData = [];
% expRCS = [];
% meanrxsig = [];


opts_car = detectImportOptions("dfltORM_Car.txt");
% opts_ped = detectImportOptions("dfltORM_Pedestrian.txt");
rcs_car_db = readmatrix("dfltORM_Car.txt",opts_car);
% rcs_ped_db = readmatrix("dfltORM_Pedestrian.txt",opts_ped);

% -------------------------------------------------------------------------------

rcs_car_pow = db2pow(rcs_car_db);
% rcs_ped_pow = db2pow(rcs_ped_db);

el = -90:1:90;
az = -180:1:180;

rcs_model_car = phased.BackscatterRadarTarget("OperatingFrequency",fc,...
    "PropagationSpeed",c,"RCSPattern",rcs_car_pow,"Model","Swerling3");
% rcs_model_ped = phased.BackscatterRadarTarget("OperatingFrequency",fc,...
%     "PropagationSpeed",c,"RCSPattern",rcs_ped_pow,"Model","Nonfluctuating");

rcs_signature_car = rcsSignature("Pattern",rcs_car_db,"Frequency",[fc fc],...
    "Elevation",el,"Azimuth",az,"FluctuationModel","Swerling3");
% rcs_signature_ped = rcsSignature("Pattern",rcs_ped_db,"Frequency",[fc fc],...
%     "Elevation",el,"Azimuth",az,"FluctuationModel","Swerling0");



% -------------------------------------------------------------------------------

scenario = drivingScenario;

roadCenters = [0 0; 10000 0];

laneSpecification = lanespec([2 2], 'Width', 3.5);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');


egoVehicle = vehicle(scenario,"ClassID",1,"RCSPattern",rcs_car_db,'Mesh',driving.scenario.carMesh);
egoVehicle.Position = [egoVehicle.RearOverhang,laneSpecification.Width(1) / (-2),0.01];
egoVehicle.Velocity = [0,0,0];
% egoVehicle.Yaw = 45;

car1 = vehicle(scenario,"ClassID",1,"FrontOverhang",1,"RCSPattern",rcs_car_db,'Mesh',driving.scenario.carMesh);
car1.Position = [egoVehicle.Length + 20 + egoVehicle.RearOverhang,laneSpecification.Width(1) / (-2),0.01];
car1.Yaw = 0;
car1.Velocity = [6,0,0];

car2 = vehicle(scenario,"ClassID",1,"FrontOverhang",1,"RCSPattern",rcs_car_db,'Mesh',driving.scenario.carMesh);
car2.Position = [egoVehicle.Length + 30 + egoVehicle.RearOverhang,laneSpecification.Width(1) / (2),0.01];
car2.Velocity = [-10,0,0];



egoWaypoints = [egoVehicle.RearOverhang,laneSpecification.Width(1) / (-2),0.01;
    250 laneSpecification.Width(1) / (-2) 0.01];
scenario.SampleTime = 0.2;

% ---------------------------------------------------------------------------------------


% vel_max_L = 100; % m/s
% vel_res_L = 0.5; % m/s
% FOV_L = 20; % deg
% doa_max_L = FOV_L / 2; % deg
% doa_res_L = 3.5; % deg




%bw = 43e6; % HZ
bw = 1.5e9
slope = 2.5e12;
pulswidth = 1.7e-5;
ts = 1.7e-5;
Nsweep = 128;
fs = 15e6;

range_res_L = bw2rangeres(bw);

range_max_FMCW = (fs * c) / (2 * slope);

waveform = phased.FMCWWaveform("SweepTime",ts,"SweepBandwidth",bw,"SampleRate",bw);
sig = step(waveform);
% spectrogram(sig,16,8,16,fs,'yaxis');
[txArray, rxArray, arrayMetrics, txPos, rxPos, ant] = helperDesignArray(lambda,fc);
Ne = size(rxArray.ElementPosition,2);



ant_gain = arrayMetrics.TxGain; %db

tx_gain = ant_gain;
tx_pk_power_db = 14.1514;
tx_pk_power = db2pow(tx_pk_power_db) * 1e-3; % W


rx_gain = arrayMetrics.RxGain;
rx_NF = 12;

% Waveform transmitter
transmitter = phased.Transmitter("PeakPower",tx_pk_power,"Gain",tx_gain);

% Radiator for transmit 
radiator = phased.Radiator("Sensor",txArray,"OperatingFrequency",fc);


% Collector for receive array
collector = phased.Collector("Sensor",rxArray,"OperatingFrequency",fc);


rx_angres = beamwidth(collector.Sensor,fc,'PropagationSpeed',c);


% Receiver preamplifier
receiver = phased.ReceiverPreamp('Gain',rx_gain,'NoiseFigure',rx_NF,'SampleRate',bw);


% Define radar
radar = radarTransceiver('Waveform',waveform,'Transmitter',transmitter,...
    'TransmitAntenna',radiator,'ReceiveAntenna',collector,'Receiver',receiver,...
    'MountingLocation',[egoVehicle.Position(1) + egoVehicle.FrontOverhang + ...
    egoVehicle.Wheelbase 0 0.3],'NumRepetitions',Nsweep);


channel = phased.FreeSpace("PropagationSpeed",c,"OperatingFrequency",fc,...
    "SampleRate",fs,"TwoWayPropagation",true);



radarConstdb = tx_pk_power_db + tx_gain + rx_gain ;
% + pow2db(lambda ^2 / (4*pi)^3)

vfd = dsp.VariableFractionalDelay;

PFA = 1e-6; 
PD = 0.9; 
S_min = detectability(PD,PFA,Nsweep,'Swerling3');
Pr_min = pow2db(physconst("Boltzmann") * systemp(rx_NF) / ts) + S_min ;
% ----------------------------------------------------------------------------------
% IDM 

IDM_a = 2; % m/s2
IDM_b = 1.5; % m/s2
IDM_delta = 4; 
% IDM_s0 = range_res_L; % m
IDM_s0 = 4;
IDM_t = 2; % s
IDM_v0 = 42;

% ----------------------------------------------------------------------------------------
% plot(scenario,"RoadCenters","on","Centerline","on");
% 
hFigure = uifigure;
hFigure.Position= [150 100 1200 600];


hPanel1 = uipanel(hFigure,'Units','Normalized','Position',[0 0 1/10 1],'Title','Scenario Plot');
hPanel2 = uipanel(hFigure,'Units','Normalized','Position',[09/40 0 1/4 1/4],'Title','Detected Targets');
hPanel3 = uipanel(hFigure,'Units','Normalized','Position',[01/10 0 1/8 1],'Title','Bird''s-Eye Plot');
hPanel4 = uipanel(hFigure,'Units','Normalized','Position',[09/40 3/4 1/8 1/4],'Title','Range-Doppler');
hPanel5 = uipanel(hFigure,'Units','Normalized','Position',[09/40 1/2 1/4 1/4],'Title','Range-Angle');
hPanel6 = uipanel(hFigure,'Units','Normalized','Position',[14/40 3/4 1/8 1/4],'Title','Range-Doppler-CFAR');
hPanel7 = uipanel(hFigure,'Units','Normalized','Position',[09/40 1/4 1/4 1/4],'Title','Range-Angle-CFAR');
hPanel8 = uipanel(hFigure,'Units','Normalized','Position',[19/40 2/3 21/40 1/3],'Title','ACC Plot (Position)');
hPanel9 = uipanel(hFigure,'Units','Normalized','Position',[19/40 1/3 21/40 1/3],'Title','ACC Plot (Velocity)');
hPanel10 = uipanel(hFigure,'Units','Normalized','Position',[19/40 0 21/40 1/3],'Title','ACC Plot (Acceleration)');

% hAxes1 = axes('Parent',hPanel1,'XLim',[0 180],'YLim',[-10 10],'DataAspectRatioMode','auto',...
%     'PlotBoxAspectRatioMode','auto','CameraViewAngleMode','auto',...
%     'Position',[0.005 0.005 1 0.96]);

hAxes1 = axes('Parent',hPanel1);
hAxes3 = axes('Parent',hPanel3,'Position',[0.1 0.005 0.8 0.96]);
lgd3 = legend(hAxes3);


hAxes4 = axes('Parent',hPanel4);
hAxes5 = axes('Parent',hPanel5);
hAxes6 = axes('Parent',hPanel6);
hAxes7 = axes('Parent',hPanel7);
hTable = uitable('Parent',hPanel2,'Units','normalized','Position',[0 0 1 1],'RowName','numbered');
hAxes8 = axes('Parent',hPanel8);
hAxes9 = axes('Parent',hPanel9);
hAxes10 = axes('Parent',hPanel10);
lgd8 = legend(hAxes8 );
lgd8.Visible = 'on';
lgd8.Location = "northwest";
xlabel(hAxes8,'Time (s)');
ylabel(hAxes8,'Position (m)');
lgd9 = legend(hAxes9 );
lgd9.Visible = 'on';
lgd9.Location = "south";
xlabel(hAxes9,'Time (s)');
ylabel(hAxes9,'Velocity (m/s)');
lgd10 = legend(hAxes10 );
lgd10.Visible = 'on';
lgd10.Location = "southwest";
xlabel(hAxes10,'Time (s)');
ylabel(hAxes10,'Acceleration (m/s2)');


rngdophandle = imagesc(hAxes4,[-55 55],[0 250], zeros(2,2));
hAxes4.YDir = 'normal';
xlim(hAxes4,[-55 55]); % m/s
ylim(hAxes4,[0 250]);
% clim(hAxes4,[10 60])
% ylabel(colorbar('peer',hAxes4),'SNR (dB)');
xlabel(hAxes4,'Radial Speed (m/s)');
ylabel(hAxes4,'Range (m)');
title(hAxes4,'Range-Doppler Image');

% 正方形显示
% rnganghandle = mesh(hAxes5,[-60 60],[0 250], zeros(2,2));
% hAxes5.YDir = 'normal';
% hAxes5.XDir = "reverse";
% ylim(hAxes5,[0 250]);
% xlim(hAxes5,[-60 60]); % m/s
% ylabel(colorbar('peer',hAxes5),'SNR (dB)');
% xlabel(hAxes5,'Angle of Arrival (deg)');
% ylabel(hAxes5,'Range (m)');
% title(hAxes5,'Range-Angle Image' );




rngdopCFARhandle = imagesc(hAxes6,[-55 55],[0 250], zeros(2,2));
hAxes6.YDir = 'normal';
xlim(hAxes6,[-55 55]); % m/s
ylim(hAxes6,[0 250]);
% clim(hAxes4,[10 60])
% ylabel(colorbar('peer',hAxes4),'SNR (dB)');
xlabel(hAxes6,'Radial Speed (m/s)');
ylabel(hAxes6,'Range (m)');
title(hAxes6,'Range-Doppler-CFAR');

% 正方形显示
% rngangCFARhandle = imagesc(hAxes7,[-60 60],[0 250], zeros(2,2));
% hAxes7.YDir = 'normal';
% hAxes7.XDir = "reverse";
% ylim(hAxes7,[0 250]);
% xlim(hAxes7,[-60 60]); % m/s
% % clim(hAxes5,[10 60])
% % ylabel(colorbar('peer',hAxes5),'SNR (dB)');
% xlabel(hAxes7,'Angle of Arrival (deg)');
% ylabel(hAxes7,'Range (m)');
% title(hAxes7,'Range-Angle-CFAR' );



restart(scenario);
scenario.StopTime = Inf;

% 画鸟瞰图

% egoCarBEP = birdsEyePlot('Parent',hAxes3,'XLimits',[-10 500],'YLimits',[-150 150]);

% % 目标的图例
% % tgtTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','red','DisplayName','target','VelocityScaling',.5);
% % 本车的图例
% % egoTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','blue','DisplayName','ego','VelocityScaling',.5);
% % 
% egoRoadPlotter = laneBoundaryPlotter(egoCarBEP);
% % plotTrack(egoTrackPlotter, egoVehicle.Position);
% egoOutlinePlotter = outlinePlotter(egoCarBEP);
% lanePlotter = laneMarkingPlotter(egoCarBEP);


velScale = 0.1;
detMrkrSize = 6;


grid(hAxes3,'on');
grid(hAxes3,'minor');
egoCarBEP = birdsEyePlot('XLim',[-1.5 100],'YLim',10*[-1 1],'Parent',hAxes3);
hold(hAxes3,'on');
p = plot(hAxes3,egoVehicle.Position(1),egoVehicle.Position(2));
hold(hAxes3,'off');
plotters = struct();

chasePlot(egoVehicle,'Parent',hAxes1,'Centerline','off','ViewHeight',70,...
    'ViewLocation', [20, 0],'ViewPitch',90);

lmPlotter = laneMarkingPlotter(egoCarBEP);
lbPlotter = laneBoundaryPlotter(egoCarBEP,'Tag','Lane boundaries');
caPlotter = coverageAreaPlotter(egoCarBEP,"FaceColor","r","EdgeColor","r");


pPlotter = detectionPlotter(egoCarBEP,'VelocityScaling',velScale,...
    'Marker','o','MarkerFaceColor','k','MarkerSize',2);
detPlotter = detectionPlotter(egoCarBEP,'VelocityScaling',velScale,...
    'Marker','o','MarkerFaceColor','g','MarkerSize',6);
olPlotter = outlinePlotter(egoCarBEP,'Tag','Truth outlines');

plotters.PosePlotter = pPlotter;
plotters.CoverageAreaPlotter = caPlotter;
plotters.OutlinePlotter = olPlotter;
plotters.LaneBoundaryPlotter = lbPlotter;
plotters.LaneMarkingPlotter = lmPlotter;
plotters.DetPlotter = detPlotter;

% Get the road boundaries and rectangular outlines
rb = roadBoundaries(egoVehicle);
% Get lane marking vertices and faces
[lmv, lmf] = laneMarkingVertices(egoVehicle);
% update the bird's-eye plotters with the road and actors
plotLaneBoundary(plotters.LaneBoundaryPlotter, rb);
plotLaneMarking(plotters.LaneMarkingPlotter, lmv, lmf);

tposes = targetPoses(egoVehicle);
pos = cell2mat(arrayfun(@(p)p.Position(:)',tposes(:),'UniformOutput',false));
vel = cell2mat(arrayfun(@(p)p.Velocity(:)',tposes(:),'UniformOutput',false));

% Add the ego
pos = [0 0 0;pos];
vel = [0 0 0;vel];

plotDetection(plotters.PosePlotter,pos(:,1:2),vel(:,1:2));

[vposition, vyaw, vlength, vwidth, voriginOffset, vcolor] = targetOutlines(egoVehicle);

[bposition, byaw, blength, bwidth, boriginOffset, bcolor] = targetOutlines(egoVehicle,'Barriers');
vposition = [vposition;bposition];
vyaw = [vyaw;byaw];
vlength = [vlength;blength];
vwidth = [vwidth;bwidth];
voriginOffset = [voriginOffset;boriginOffset];
vcolor = [vcolor;bcolor];

plotOutline(plotters.OutlinePlotter, vposition, vyaw, vlength, vwidth, ...
    'OriginOffset', voriginOffset, 'Color', vcolor);


thisSensor = radar;
% Create detection plotter
isTracks = isa(thisSensor,'matlabshared.tracking.internal.fusion.BaseRadarDataGenerator') ...
    && startsWith(lower(thisSensor.TargetReportFormat),'T');
if isTracks
    pltr = trackPlotter(egoCarBEP,'HistoryDepth',7, ...
        'MarkerFaceColor','r');
    % plotters.DetectionPlotters(1).SensorIndex = thisSensor.SensorIndex;
else
    pltr = trackPlotter(egoCarBEP,'HistoryDepth',0, ...
        'MarkerFaceColor','r','Marker','o','MarkerSize',detMrkrSize);
    % plotters.DetectionPlotters(1).SensorIndex = thisSensor.SensorIndex;
end
plotters.DetectionPlotters(1).Plotter = @(dets)plotDets(pltr,thisSensor,dets);
lgd3.Visible = 'off';
xlabel(hAxes3,[]);
ylabel(hAxes3,[]);
% ---------------------------------------------------------------------------------
% range-doppler && range-angle
azfov = 120;
numBeams = ceil(azfov/rx_angres);
numBeams = floor(numBeams/2)*2+1;
azgrid = azfov*linspace(-0.5,0.5,numBeams);

txSV = phased.SteeringVector('SensorArray',radiator.Sensor, 'PropagationSpeed',c);
txSteer = txSV(fc,azgrid);


rxSV = phased.SteeringVector('SensorArray',collector.Sensor,'PropagationSpeed',c);
rxSteer = rxSV(fc,azgrid);

vxSteer = helperVirtualSteeringVector(txSteer,rxSteer);


Nft = waveform.SweepTime * waveform.SampleRate; % Number of fast-time samples
Nst = Nsweep;                                 % Number of slow-time samples
NRxElmnts = getNumElements(rxArray);
numFrames = numel(azgrid);
tFrame = ts*Nsweep;

Nr = 2^(nextpow2(Nft));                         % Number of range samples 
Nd = 2^nextpow2(Nst);                         % Number of Doppler samples 

rngdopresp = phased.RangeDopplerResponse('PropagationSpeed',c,...
    'DopplerOutput','Speed','OperatingFrequency',fc,'SampleRate',bw,...
    'RangeMethod','FFT', ...
    'SweepSlope',slope,'DopplerFFTLengthSource','Property', ...
    'DopplerFFTLength',Nd,'DopplerWindow','Custom','CustomDopplerWindow',@hanning);


% rngangresp = phased.RangeAngleResponse("SensorArray",rxArray, ...
%     "RangeMethod","FFT","OperatingFrequency",fc,"SampleRate",bw, ...
%     "SweepSlope",slope,"NumAngleSamples",91);

beamformer = phased.PhaseShiftBeamformer('SensorArray',rxArray,...
    'PropagationSpeed',c,'OperatingFrequency',fc,'Direction',[azgrid;0 * azgrid]);

% ---------------------------------------------------------------------------------------------------
% CFAR

% Guard cell and training regions for range dimension
nGuardRng = 3;
nTrainRng = 3;
nCUTRng = 1+nGuardRng+nTrainRng;

% Guard cell and training regions for Doppler dimension
dopOver = round(Nd/Nsweep);
nGuardDop = 3*dopOver;
nTrainDop = 3*dopOver;
nCUTDop = 1+nGuardDop+nTrainDop;

% Guard cell and training regions for angle dimension
nGuardAng = 3;
nTrainAng = 3;
nCUTAng = 1+nGuardAng+nTrainAng;

cfarRngDop = phased.CFARDetector2D('GuardBandSize',[nGuardRng nGuardDop],...
    'TrainingBandSize',[nTrainRng nTrainDop],...
    'ThresholdFactor','Custom','CustomThresholdFactor', ...
    db2pow(12),...
    'NoisePowerOutputPort',true,'OutputFormat','Detection index','ThresholdOutputPort',true);

% cfarRngAng = phased.CFARDetector2D('GuardBandSize',[nGuardRng nGuardAng],...
%     'TrainingBandSize',[nTrainRng nTrainAng],...
%     'ThresholdFactor','Custom','CustomThresholdFactor', ...
%     db2pow(20),...
%     'NoisePowerOutputPort',true,'OutputFormat','CUT result','ThresholdOutputPort',true);



% -------------------------------------------------------------------------------
% 参数估计


clustererRngDop = clusterDBSCAN( 'MinNumPoints',2);
clustererRngAng = clusterDBSCAN( 'MinNumPoints',2);

rmsRng = sqrt(12) * range_res_L;

rgestimator = phased.RangeEstimator(...
    'ClusterInputPort',true, ...
    'VarianceOutputPort',true,...
    'NoisePowerSource','Input port', ...
    'RMSResolution',range_res_L);

rrestimator = phased.DopplerEstimator(...
    'ClusterInputPort',true, ...
    'VarianceOutputPort',true, ...
    'NoisePowerSource','Input port', ...
    'NumPulses',Nsweep);

decflag = 0;
% -------------------------------------------------------------------------------------------------------
% snr = [];
% NSample = 0;
% NDetect = 0;
% NFalse = 0;
% 
% expdetidx = zeros(size(idxCFAR,2),1);
% expthre = zeros(size(idxCFAR,2),1);
% expnpw = zeros(size(idxCFAR,2),1);
% expfft = zeros(Nr,1);


rngidel = [];
velidel = [];
rngest = [];
velest = [];
azidel = [];
rcsidel = [];
rcsest = [];
azest = [];

hegovel = animatedline(hAxes9,"Color",'b','LineWidth',1.25,...
    'DisplayName','Ego Velocity','LineStyle','-');
hegoacc = animatedline(hAxes10,"Color",'#FF6347','LineWidth',1.25,...
    'DisplayName','Ego Acceleration','LineStyle','-');

htgtpos = animatedline(hAxes8,"Color",'r','LineWidth',1.25,...
    'DisplayName','Detected Target Position','LineStyle','--');
htgtvel = animatedline(hAxes9,"Color",'#FF00FF','LineWidth',1.25,...
    'DisplayName','Detected Target Velocity','LineStyle','--'); 

% htgtvel_ideal = animatedline(hAxes9,'LineWidth',1.25,...
%     'DisplayName','Detected Target Velocity','LineStyle','--'); 

htgtpos_gruand = animatedline(hAxes8,"Color",'#0072BD','LineWidth',1.25,...
    'DisplayName','Target Position (Ground Truth)','LineStyle','-.');
htgtvel_gruand = animatedline(hAxes9,"Color",'g','LineWidth',1.25,...
    'DisplayName','Target Velocity (Ground Truth)','LineStyle','-.');
hegoacc_gruand = animatedline(hAxes10,"Color",'#4DBEEE','LineWidth',1.25,...
    'DisplayName','Ego Acceleration (Ideal)','LineStyle','--');

fa = 0;
egoxt_ = egoVehicle.Position(1);
egovt_ = egoVehicle.Velocity(1);
while advance(scenario)
    % Xframes = NaN(Nft,NRxElmnts,Nst,numFrames);
    time = scenario.SimulationTime;
    % NSample = NSample + 1;
%     if (NSample == 1e6+1)
%         break;
%     end
    tgtRelPoses = targetPoses(egoVehicle);
    Ntgt = numel(tgtRelPoses);
    tgtpos = zeros(3,Ntgt);
    tgtrng = zeros(1,Ntgt);
    tgtang = zeros(2,Ntgt);
    tgtYaw = zeros(1,Ntgt);
    tgtvel = zeros(3,Ntgt);
    for i = 1:numel(tgtRelPoses)
        boundingPose = local2globalcoord([3.8,2.6,1.4,0.2,-1,3.8,-1,3.8,2.6,1.4,0.2,-1;...
         -0.9,-0.9,-0.9,-0.9,-0.9,0,0,0.9,0.9,0.9,0.9,0.9;0 0 0 0 0 0 0 0 0 0 0 0],...
            "rr",tgtRelPoses(i).Position');
        [pointrng,pointang]= rangeangle(boundingPose,[egoVehicle.FrontOverhang +...
            egoVehicle.Wheelbase;0;0]);
        [min_tgtrng, min_index] = min(pointrng);
        tgtrng(i) = min_tgtrng;
        if abs(pointang(1,min_index)) <= 90
            tgtang(:,i) = pointang(:,min_index);
            tgtYaw(i) = tgtRelPoses(i).Yaw;
            tgtvel(:,i) = tgtRelPoses(i).Velocity';
            tgtpos(:,i) = tgtRelPoses(i).Position';
        else
            tgtang(:,i) = [];
            tgtrng(i) = [];
            tgtYaw(i) = [];
            tgtvel(:,i) = [];
            tgtpos(:,i) = [];
        end
        
    end

%------------------------------------------------------------------------------
% 毫米波传播
    % sv_tgt = txSV(fc,tgtang);
    % svv_tgt = sum(sv_tgt,1);
    % arrayEffect_tgt = rxSV(fc,tgtang);

    tgtRCS_db = value(rcs_signature_car,(tgtYaw+9) + tgtang(1,:),tgtang(2,:),fc);
    % tgtRCS_db = -10 + time-0.2;
    % tgtRCS_db = 3;

    tgtRCS = aperture2gain(db2pow(tgtRCS_db),lambda);
    % rcsidel = [rcsidel tgtRCS_db];

    R_max_tgt = radareqrng(lambda,S_min,radar.Transmitter.PeakPower,ts,"Gain",[radar.Transmitter.Gain, ...
        radar.Receiver.Gain],"RCS",db2pow(tgtRCS_db),"Ts",systemp(rx_NF));

    [car_path,Npathtgt] = helperSpacePaths(tgtpos,tgtrng,tgtang,tgtvel,...
        R_max_tgt',lambda,tgtRCS);
    % for iFrm = 1:numFrames
    %     rx_sig = radar(car_path,time+(iFrm-1)*tFrame,conj(txSteer(:,iFrm)));
    %     Xframes(:,:,:,iFrm) = rx_sig;
    % end
    rx_sig = radar(car_path,time);
    Xframes = rx_sig;
% ----------------------------------------------------------------------------
% RCS估计
    % Npath = size(car_path,2);
    % 
    % 
    % nDelay = zeros(1,size(car_path,2));
    % tx_sig = zeros(size(sig,1),Npath);
    % for i = 1:Npath
    %     tx_sig(:,i) = sig;
    %     nDelay(:,i) = car_path(i).PathLength / c * fs;
    % end
    % 
    % fDelay = nDelay-fix(nDelay);
    % startidx = ceil(nDelay);
    % endidx = size(sig,1);
    % channel_vfd = vfd(tx_sig,fDelay);
    % channel_sig_out = zeros(size(tx_sig));
    % for i =1:Npath
    %     channel_sig_out(startidx(i):endidx,i) = channel_vfd(1:endidx+1-startidx(i),i);
    % end
    % 
    % pow_ratio_channel = pow2db(abs(channel_sig_out) .^ 2 ./ abs(tx_sig) .^ 2);
    % 
    % pow_ratio = pow2db(abs(rx_sig(:,:,1) * pinv(arrayEffect_tgt.')).^2) - pow2db(abs(sig * svv_tgt).^2)...
    %     - pow_ratio_channel - radar.Transmitter.Gain - radar.Receiver.Gain...
    %     - pow2db(radar.Transmitter.PeakPower) + 2 * fspl(tgtrng,lambda);
    % 
    % for i = 1:Npath
    %     rcs_tgt_db = pow2db(gain2aperture(pow_ratio(:,i),lambda));
    %     rcs_tgt_est = mean(rcs_tgt_db(rcs_tgt_db ~= -inf & rcs_tgt_db ~= inf & ~isnan(rcs_tgt_db)));
    % end

% -----------------------------------------------------------------------------------------------------------------
% 雷达数字信号处理
    [Xrngdop,rggrid,rrgrid] = helperRangeDopplerProcessing(Xframes,radar,rngdopresp);
    Xrngdopdb = pow2db(abs(Xrngdop).^2);
    [~,iMax] = max(Xrngdopdb(:));
    [~,~,~,iFrm] = ind2sub(size(Xrngdopdb),iMax);
    
    
    % 第一种beamformer
    % [Xbmrngdop] = helperVirtualBeamform(txSteer,vxSteer,Xrngdop);

    % 第二种beamformer
    Xbmrngdop = permute(Xrngdop,[1 3 2]);
    Xbmrngdop = reshape(Xbmrngdop,size(Xrngdop,1)*Nd,Ne);
    Xbmrngdop = beamformer(Xbmrngdop);
    Xbmrngdop = reshape(Xbmrngdop,size(Xrngdop,1),Nd,[]);
    Xbmrngdop = permute(Xbmrngdop,[1 3 2]);

    % 第三种，不用beamformer
    % Xbmrngdop = Xrngdop;

    Xbmpow = abs(Xbmrngdop).^2;
    Xbmdb = pow2db(Xbmpow);


    iRngCUT = find(rggrid>=0);
    iRngCUT = iRngCUT((iRngCUT>=nCUTRng)&(iRngCUT<=size(rggrid,1)-nCUTRng+1));
    iDopCUT = nCUTDop:(Nd-nCUTDop+1);
    [iRng,iDop] = meshgrid(iRngCUT,iDopCUT);
    idxCFARRngDop = [iRng(:) iDop(:)]';


    [detRngDop,threRngDop,noisepwrRngDop] = cfarRngDop(squeeze(max(Xbmpow,[],2)),idxCFARRngDop);

    epsilonRngDop = clusterDBSCAN.estimateEpsilon(detRngDop',2,10);
    clustererRngDop.Epsilon = epsilonRngDop;

    [~,clusterIDsRngDop] = clustererRngDop(detRngDop');

    clusterIDsRngDop = clusterIDsRngDop(:)';

    [meas,noise,snrdB,detRngAng] = helperMeasurementEstimation(Xbmrngdop,noisepwrRngDop,...
        radar,azgrid,rggrid,rrgrid,clusterIDsRngDop,detRngDop,rgestimator,rrestimator);

    dets = helperAssembleDetections(radar,time,meas,noise,snrdB);


    Xrngdopdb_2 = squeeze(max(Xrngdopdb,[],2));
    Xrngdopdb_CFAR = min(Xrngdopdb_2,[],'all') * ones(size(Xrngdopdb_2));
    Xrngdopdb_CFAR(sub2ind(size(Xrngdopdb_CFAR),detRngDop(1,:),detRngDop(2,:))) = Xrngdopdb_2(sub2ind(size(Xrngdopdb_CFAR),detRngDop(1,:),detRngDop(2,:))); 

    Xrngangdb_2 = squeeze(max(Xbmdb,[],3));
    Xrngangdb_CFAR = min(Xrngangdb_2,[],'all') * ones(size(Xrngangdb_2));
    Xrngangdb_CFAR(sub2ind(size(Xrngangdb_CFAR),detRngAng(1,:),detRngAng(2,:))) = Xrngangdb_2(sub2ind(size(Xrngangdb_CFAR),detRngAng(1,:),detRngAng(2,:))); 
    
    if size(meas,2) > 1
        fa = fa + 1;
    end
    % azest = [azest meas(1)];
    % rngest = [rngest meas(2)];
    % velest = [velest meas(3)];
    % rcsest = [rcsest rcs_tgt_est];
    % azidel = [azidel tgtang(1,1)];
    % rcsidel = [rcsidel tgtRCS_db];

% ---------------------------------------------------------------------------------------------------------------------------
% ACC算法 + 车辆位置更新
    ACCtgts = [];
    kk = 0;
    for i =1:size(meas,2)
        laterDis = abs(sin(deg2rad(meas(1,i))) * meas(2,i));
        if laterDis + egoVehicle.Width/2 <= (3.5)/2
            kk = kk+1;
            ACCtgts(1:3,kk) = meas(:,i);
            ACCtgts(4,kk) = cos(deg2rad(meas(1,i))) * meas(2,i);
        end
    end

    egov0 = egoVehicle.Velocity(1);
    egox0 = egoVehicle.Position(1);

    % s = car1.Position(1) - egox0 - egoVehicle.Length;
    % v_delta = egov0 - car1.Velocity(1);
    if size(ACCtgts,2) > 0
        [s,tgtId] = min(ACCtgts(4,:));
        s_ = car1.Position(1) - egoxt_- egoVehicle.Length;
        v_delta = ACCtgts(3,tgtId);
        v_delta_ = egovt_ - car1.Velocity(1);
    else
        s = inf;
        s_ = inf;
        v_delta = 0;
        v_delta_ = 0;
    end
    addpoints(htgtpos,time,s);
    addpoints(htgtvel,time,egov0-v_delta);


    addpoints(hegovel,time,egov0);

    addpoints(htgtpos_gruand,time,car1.Position(1) - egox0- egoVehicle.Length);
    addpoints(htgtvel_gruand,time,car1.Velocity(1));
    rngidel = [rngidel car1.Position(1) - egox0- egoVehicle.Length];
    egoacc = IDM_a * (1 - (egov0 / IDM_v0) ^ IDM_delta...
        - ( (IDM_s0 + max(0,egov0 * IDM_t + (egov0 * v_delta) / (2 * sqrt(IDM_a * IDM_b)) )) / s) ^ 2);
    egoacc_ = IDM_a * (1 - (egovt_ / IDM_v0) ^ IDM_delta...
        - ( (IDM_s0 + max(0,egovt_ * IDM_t + (egovt_ * v_delta_) / (2 * sqrt(IDM_a * IDM_b)) )) / s_) ^ 2);
    

    if egoacc < -5
        egoacc = -5;
    elseif egoacc > 2
        egoacc = 2;
    end
    if egoacc_ < -5
        egoacc_ = -5;
    elseif egoacc_ > 2
        egoacc_ = 2;
    end

    egoacc = 0;
    % egov0 = 0;
    % egovt = 0;


    egovt = egov0 + egoacc * scenario.SampleTime;
    egovt_ = egovt_ + egoacc_ * scenario.SampleTime;

    if egovt < 0
        egovt = 0;
        egoacc = 0;
        egoacc_ = 0;
    end
    egoacc_ = 0;
    addpoints(hegoacc,time,egoacc);
    addpoints(hegoacc_gruand,time,egoacc_);
    % addpoints(htgtvel_ideal,time,egovt_);

    egoxt = egox0 + egov0 * scenario.SampleTime + 0.5 * egoacc * scenario.SampleTime ^ 2;
    egoxt_ = egoxt_ + egovt_ * scenario.SampleTime + 0.5 * egoacc_ * scenario.SampleTime ^ 2;

    egoVehicle.Position(1) = egoxt;
    egoVehicle.Velocity(1) = egovt;

    
    
    car1v0 = car1.Velocity(1);
    car1x0 = car1.Position(1);
    % if abs(egovt - 30) <= 0.1 && decflag == 0 && time > 1 
    %     decflag = time;
    % end

    % if decflag >0 && time -decflag >= 10
    %     car1Acc = -1.5;
    % else
    %     car1Acc = 0;
    % end

    % if car1v0 <= 27
    %     car1Acc = 1.5;
    % else
    %     car1Acc = 0;
    % end
    car1Acc = 0;
    car1vt = car1v0 + car1Acc * scenario.SampleTime;
    if car1vt<=0
        car1Acc = 0;
        car1vt = 0;
    end
    car1xt = car1x0 + car1v0 * scenario.SampleTime + 0.5 * car1Acc * scenario.SampleTime ^ 2;
    car1.Position(1) = car1xt;
    car1.Velocity(1) = car1vt;


    % car2Acc = 0;
    % car2v0 = car2.Velocity(1);
    % car2x0 = car2.Position(1);
    % car2vt = car2v0 + car2Acc * scenario.SampleTime;
    % car2xt = car2x0 + car2v0 * scenario.SampleTime + 0.5 * car2Acc * scenario.SampleTime ^ 2;
    % car2.Position(1) = car2xt;
    % car2.Velocity(1) = car2vt;



% %     cla(hAxes7);
% %     text(hAxes7,0.2,0.2,"false:"+100*NFalse/NSample/size(idxCFAR,2));
% %     text(hAxes7,0.2,0.6,"detect:"+100*NDetect/NSample);
% 

% %     if(mod(NSample,500) ==0)
%         pfa = NFalse/NSample/size(idxCFAR,2);
%         pd = NDetect/NSample;
% %     end

% -------------------------------------------------------------------------------------
% % 图表更新，先注释掉

    [lmv, lmf] = laneMarkingVertices(egoVehicle);
    plotLaneMarking(plotters.LaneMarkingPlotter,lmv,lmf);

    rb = roadBoundaries(egoVehicle);
    plotLaneBoundary(plotters.LaneBoundaryPlotter, rb);

    [vposition, vyaw, vlength, vwidth, voriginOffset, vcolor] = targetOutlines(egoVehicle);
    plotOutline(plotters.OutlinePlotter, vposition, vyaw, vlength, vwidth, ...
        'OriginOffset', voriginOffset, 'Color', vcolor);

    plotCoverageArea(plotters.CoverageAreaPlotter,[egoVehicle.FrontOverhang + egoVehicle.Wheelbase 0],300,0,120);

    pos = cell2mat(arrayfun(@(p)p.Position(:)',tgtRelPoses(:),'UniformOutput',false));
    vel = cell2mat(arrayfun(@(p)p.Velocity(:)',tgtRelPoses(:),'UniformOutput',false));

    % Add the ego
    pos = [0 0 0;pos];
    vel = [0 0 0;vel];

    plotDetection(plotters.PosePlotter,pos(:,1:2),vel(:,1:2));
    
 



    detsEgo = matlabshared.tracking.internal.fusion.objectDetection.toParentFrame(dets);
    if ~isempty(detsEgo)
        detpos = cell2mat(cellfun(@(d)d.Measurement(1:3),detsEgo(:)','UniformOutput',false));
        plotDetection(plotters.DetPlotter,detpos(1:2,:)');
    end

    rngdophandle.YData = rggrid;
    rngdophandle.XData = rrgrid;
    rngdophandle.CData = squeeze(max(Xrngdopdb,[],2));

    % 扇形显示
    [xgrid,ygrid] = meshgrid(azgrid,rggrid);
    [xgrid,ygrid] = pol2cart(deg2rad(xgrid),ygrid);
    % Transform grid into ego's body frame
    [rot,off] = helperSensorFrame(radar);
    xgrid(:) = rot(1,1:2)*[xgrid(:) ygrid(:)]' + off(1);
    ygrid(:) = rot(2,1:2)*[xgrid(:) ygrid(:)]' + off(2);
    rngangMesh = mesh(hAxes5,xgrid,ygrid,-10*ones(size(xgrid)),squeeze(max(Xbmdb,[],3)));
    set(rngangMesh,'FaceColor','interp');
    view(hAxes5,-90,90);
    ylim(hAxes5,[-250 250]);
    xlim(hAxes5,[5 250]); % m/s

    % 正方形显示
    % rnganghandle.YData = rggrid;
    % rnganghandle.XData = azgrid;
    % rnganghandle.CData = squeeze(max(Xbmdb,[],3));

    rngdopCFARhandle.YData = rggrid;
    rngdopCFARhandle.XData = rrgrid;
    rngdopCFARhandle.CData = Xrngdopdb_CFAR;

    % 正方形显示
    % rngangCFARhandle.YData = rggrid;
    % rngangCFARhandle.XData = azgrid;
    % rngangCFARhandle.CData = Xrngangdb_CFAR;

    % 扇形显示
    rngangMeshCFAR = mesh(hAxes7,xgrid,ygrid,-10*ones(size(xgrid)),Xrngangdb_CFAR);
    set(rngangMeshCFAR,'FaceColor','interp');
    view(hAxes7,-90,90);
    ylim(hAxes7,[-250 250]);
    xlim(hAxes7,[5 250]); % m/s

    % axRgAz = helperPlotRangeAzimuthDetections(egoCarBEP,Xrngangdb_CFAR,noisepwrRngDop,radar,azgrid,rggrid,dets);
    tableData = array2table(meas','VariableNames',{'Angle','Range','RelVelocity'});
    hTable.Data = tableData;


    drawnow;
    % 
    % % 
    % % plot(hAxes6,rngidel);
    % % hold(hAxes6,"on");
    % % plot(hAxes6,rngest);
    % % hold(hAxes6,"off");
    % % plot(hAxes7,abs(velidel));
    % % hold(hAxes7,"on");
    % % plot(hAxes7,velest);
    % % hold(hAxes7,"off");
% -------------------------------------------
% RCS估计用的

    release(vfd);
 
end


%%
function plotDets(pltr,sensor,dets)

    % Check if this is a 2D plotter
    is2D = isa(pltr,'driving.birdsEyePlot.DetectionPlotter') || isa(pltr,'driving.birdsEyePlot.TrackPlotter');
    
    % Plot on all of the copied axes too
    hndls = findobj(groot,'Type','Line','DisplayName',pltr.DisplayName);
    if isempty(dets)
        for m = 1:numel(hndls)
            this = hndls(m);
            if is2D
                % BEP is 2D
                set(this,'XData',zeros(1,0),'YData',zeros(1,0));
            else
                set(this,'XData',zeros(1,0),'YData',zeros(1,0),'ZData',zeros(1,0));
            end
        end
        %clearData(pltr);
    else
        [rot,off] = helperSensorFrame(sensor);
        if isa(sensor,'matlabshared.tracking.internal.fusion.BaseRadarDataGenerator')
            fmt = lower(char(sensor.TargetReportFormat));
            if fmt(1)=='t' % Tracks
                pos = cell2mat(arrayfun(@(t)t.State(1:2:end),dets(:)','UniformOutput',false));
                pos = pos(1:2:end,:);
            else % Detections
                dets = matlabshared.tracking.internal.fusion.objectDetection.rectangular(dets);
                pos = cell2mat(cellfun(@(d)d.Measurement,dets(:)','UniformOutput',false));
                crds = lower(char(sensor.DetectionCoordinates));
                if crds(1)=='s' % Sensor's frame
                    pos = rot*pos(1:3,:) + off;
                end
            end
        else
            dets = matlabshared.tracking.internal.fusion.objectDetection.rectangular(dets);
            crds = lower(char(sensor.DetectionCoordinates));
            if crds(1)=='e' % Ego
                pos = cell2mat(cellfun(@(d)d.Measurement,dets(:)','UniformOutput',false));
            else
                pos = cell2mat(cellfun(@(d)d.Measurement,dets(:)','UniformOutput',false));
                pos = rot*pos(1:3) + off;
            end
        end
        
        for m = 1:numel(hndls)
            this = hndls(m);
            if is2D
                % BEP is 2D
                set(this,'XData',pos(1,:),'YData',pos(2,:));
            else
                set(this,'XData',pos(1,:),'YData',pos(2,:),'ZData',pos(3,:));
            end
        end
        
        %if ismethod(pltr,'plotTrack')
        %    plotTrack(pltr,pos(1:3,:)');
        %else
        %    plotDetection(pltr,pos(1:3,:)');
        %end
    end
end