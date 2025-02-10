function [paths,Ntar] = helperSpacePaths(tgt_pos_rel,tgt_rng_rel,tgt_ang_rel,tgt_vel_rel,tgt_rng_max,lambda,rcsgain)

% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

    Ntargets = size(tgt_rng_rel,2);

    % path loss
    pathloss = fspl(tgt_rng_rel,lambda);
%     rainloss = rainpl(tgt_range_rel,fc,10);
    
    % doppler
    speeddop = speed2dop(radialspeed(tgt_pos_rel, tgt_vel_rel),lambda);

    
    
    
    paths = repmat(struct(...
        'PathLength', zeros(1, 1), ...
        'PathLoss', zeros(1, 1), ...
        'ReflectionCoefficient', zeros(1,1), ...
        'AngleOfDeparture', zeros(2, 1), ...
        'AngleOfArrival', zeros(2, 1), ...
        'DopplerShift', zeros(1, 1)),...
        1,Ntargets);
    
    pathindex = 0;
    for m = 1:Ntargets

        % two way
        if tgt_rng_rel(:,m) > tgt_rng_max(:,m)
            paths(pathindex+1) = [];
            continue;
        else
            plength = 2 * tgt_rng_rel(:,m);
            pathindex = pathindex + 1;
        end
        
        ploss = 2 * pathloss(:,m);
%         rainloss = 2* rainloss;
        dop = 2 * speeddop(m,:);
        
        paths(pathindex).PathLength = plength;
%         paths(m).PathLoss = ploss + rainloss;
        paths(pathindex).PathLoss = ploss;
        paths(pathindex).ReflectionCoefficient = db2mag(rcsgain(m,:));
        paths(pathindex).AngleOfDeparture = tgt_ang_rel(:,m);
        paths(pathindex).AngleOfArrival = tgt_ang_rel(:,m);
        paths(pathindex).DopplerShift = dop;
    end
    Ntar = size(paths,2);
end
