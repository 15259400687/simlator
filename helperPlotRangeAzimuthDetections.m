function axRgAz = helperPlotRangeAzimuthDetections(bep,Xbmfrngdop,estnoiselvldB,xcvrLLR,azgrid,rggrid,dets)
% This function is for demo purposes only and may be removed in the future
% axRgAz = helperPlotRangeAzimuthDetections(bep,Xbmfrngdop,estnoiselvldB,threshdB,xcvrLLR,azgrid,rggrid,dets)

%   Copyright 2021-2022 The MathWorks, Inc.

figRgAz = figure('Name','Range-Angle Image');
% clf(hPanel);
axRgAz = axes(figRgAz);
figRgAz.Visible = 'on';
[xgrid,ygrid] = meshgrid(azgrid,rggrid);
[xgrid,ygrid] = pol2cart(deg2rad(xgrid),ygrid);

% Transform grid into ego's body frame
[rot,off] = helperSensorFrame(xcvrLLR);
xgrid(:) = rot(1,1:2)*[xgrid(:) ygrid(:)]' + off(1);
ygrid(:) = rot(2,1:2)*[xgrid(:) ygrid(:)]' + off(2);

% XsnrdB = pow2db(abs(Xbmfrngdop).^2)-min(pow2db(estnoiselvldB));
% XrgazdB = max(XsnrdB,[],3);
XrgazdB = Xbmfrngdop-min(pow2db(estnoiselvldB));

set(mesh(axRgAz,xgrid,ygrid, ...
    -10*ones(size(xgrid)), ...  % set the mesh z-values below the bird's eye plot
    XrgazdB), ...               % SNR color values for the mesh
    'FaceColor','interp','DisplayName','SNR (dB)');
view(axRgAz,-90,90);

% clim(axRgAz,[min(pow2db(threshdB)) min(max(clim(axRgAz)),min(pow2db(threshdB))+30)]);

xlabel(axRgAz,'X (m)'); ylabel(axRgAz,'Y (m)');
grid(axRgAz,'on'); grid(axRgAz,'minor');
set(axRgAz,'Layer','top');

% Copy BEP into new figure
shh = get(groot,'ShowHiddenHandles');
set(groot,'ShowHiddenHandles','on');
ax = bep.Parent;
copyobj(ax.Children,axRgAz);
set(groot,'ShowHiddenHandles',shh);

zlim(axRgAz,'auto');
xlim(axRgAz,[0 100]); ylim(axRgAz,10*[-1 1]);
% xlim(axRgAz,[20 40]); ylim(axRgAz,5*[-1 1]);
% axis(axRgAz,'equal');

% Plot the position estimates
pltr = bep.Plotters(end);
detsEgo = matlabshared.tracking.internal.fusion.objectDetection.toParentFrame(dets);
pos = cell2mat(cellfun(@(d)d.Measurement(1:3),detsEgo(:)','UniformOutput',false));

wasHeld = ishold(axRgAz);
hold(axRgAz,'on');



% mrkrFaceClr = lines(5);
mrkrFaceClr = 'g';
mrkrFaceClr = mrkrFaceClr(end,:);
plot(axRgAz,pos(1,:),pos(2,:),'o','DisplayName','Signal-level detections','Marker',pltr.Marker,'MarkerSize',pltr.MarkerSize,'MarkerEdgeColor','k','MarkerFaceColor',mrkrFaceClr);
if ~wasHeld
    hold(axRgAz,'off');
end

% hcbar = colorbar(axRgAz);
% set(get(hcbar,'YLabel'),'String','SNR (dB)');

% leg = legend(axRgAz);
% leg.Location = 'northeast';
% leg.AutoUpdate = 'off';
% leg.String{3} = 'Radar FoV';
% leg.String{4} = 'Radar detections';


end
