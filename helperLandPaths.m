function paths = helperLandPaths(tgt_pos_rel,tgt_rng_rel,tgt_ang_rel,tgt_vel_rel,lambda,...
    rcsgain,r_land,angle_land,rcs_land,tgt_rng_max,R_max_land)

% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.


% ------------------------------------------------------------------------
% 
    Ntargets = size(tgt_rng_rel,2);
    

    % path loss
    tgtploss = fspl(tgt_rng_rel,lambda);
    
    % doppler
    dop = speed2dop(radialspeed(tgt_pos_rel, tgt_vel_rel),lambda);

% ------------------------------------------------------------------------

    
% ------------------------------------------------------------------------

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


        ploss = 2 * tgtploss(:,m);
        pdop = 2 * dop(m,:);
        
        paths(pathindex).PathLength = plength;
        paths(pathindex).PathLoss = ploss;
%         paths(m).PathLoss = ploss;
        paths(pathindex).ReflectionCoefficient = db2mag(rcsgain(m,:));
        paths(pathindex).AngleOfDeparture = tgt_ang_rel(:,m);
        paths(pathindex).AngleOfArrival = tgt_ang_rel(:,m);
        paths(pathindex).DopplerShift = pdop;
    end

    % path loss
    landploss = fspl(r_land,lambda);
    
    % doppler
    dop = 0;
    Npaths = size(paths,2);

    if r_land <= R_max_land

        plength = 2 * r_land;
    

        ploss = 2 * landploss;
        pdop = 2 * dop;
    
    
        paths(Npaths+1).PathLength = plength;
        paths(Npaths+1).PathLoss = ploss;
        paths(Npaths+1).ReflectionCoefficient = db2mag(rcs_land);
        paths(Npaths+1).AngleOfDeparture = angle_land;
        paths(Npaths+1).AngleOfArrival = angle_land;
        paths(Npaths+1).DopplerShift = pdop;
    end

end
