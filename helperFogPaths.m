% function [paths,svv,arrayEffect] = helperFogPaths(tgt_pos_rel,tgt_rng_rel,tgt_ang_rel,tgt_vel_rel,lambda,rcsgain,...
%     rcsrate_fog_pow,fc,tgt_rng_max,R_max_fog,fograte,fog_az,r_res,txArray,rxArray)
function [paths] = helperFogPaths(tgt_pos_rel,tgt_rng_rel,tgt_ang_rel,tgt_vel_rel,lambda,rcsgain,...
    rcsrate_fog_pow,fc,tgt_rng_max,R_max_fog,fograte,fog_az,r_res)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

% ------------------------------------------------------------------------
% 
    Ntargets = size(tgt_rng_rel,2);
    

    % path loss
    tgtploss = fspl(tgt_rng_rel,lambda);
    tgtfogloss = fogpl(tgt_rng_rel,fc,20,fograte);

    % doppler
    dop = speed2dop(radialspeed(tgt_pos_rel, tgt_vel_rel),lambda);

% ------------------------------------------------------------------------
% backscatter of fog
% rcs_signature_fog
    rcs_fog  = [];
    path_fog = [];
    if R_max_fog >= r_res
        fog_range_rel = tgt_rng_rel(abs(tgt_ang_rel(1,:)) <= fog_az);
        Ntargets_fog = size(fog_range_rel,2);
        if Ntargets_fog == 0
            for i = 1:ceil(R_max_fog / r_res)-1
                r_fog = i * r_res;
                mean_fog_rcs_db = pow2db(rcsrate_fog_pow * r_fog ^ 2);
                rcs_signature_fog = rcsSignature("Pattern",mean_fog_rcs_db,...
                    "Frequency",[fc fc],"FluctuationModel","Swerling1");
                rcs_fog = [rcs_fog rcs_signature_fog];
                path_fog = [path_fog i * r_res];
            end
        else 
            if max(fog_range_rel) < R_max_fog
                for i = 1:ceil(max(fog_range_rel) / r_res)-1
                    r_fog = i * r_res;
                    mean_fog_rcs_db = pow2db(rcsrate_fog_pow * r_fog ^ 2);
                    rcs_signature_fog = rcsSignature("Pattern",mean_fog_rcs_db,...
                        "Frequency",[fc fc],"FluctuationModel","Swerling1");
                    rcs_fog = [rcs_fog rcs_signature_fog];
                    path_fog = [path_fog i * r_res];
                end
            else
                for i = 1:ceil(R_max_fog / r_res)-1
                    r_fog = i * r_res;
                    mean_fog_rcs_db = pow2db(rcsrate_fog_pow * r_fog ^ 2);
                    rcs_signature_fog = rcsSignature("Pattern",mean_fog_rcs_db,...
                        "Frequency",[fc fc],"FluctuationModel","Swerling1");
                    rcs_fog = [rcs_fog rcs_signature_fog];
                    path_fog = [path_fog i * r_res];
                end
            end
        end
    end
    
    
% ------------------------------------------------------------------------

    Nfog = size(rcs_fog,2);
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
        pfogloss = 2 * tgtfogloss(m,:);
        pdop = 2 * dop(m,:);
        
        paths(pathindex).PathLength = plength;
        paths(pathindex).PathLoss = ploss + pfogloss;
        paths(pathindex).ReflectionCoefficient = db2mag(rcsgain(m,:));
        paths(pathindex).AngleOfDeparture = tgt_ang_rel(:,m);
        paths(pathindex).AngleOfArrival = tgt_ang_rel(:,m);
        paths(pathindex).DopplerShift = pdop;
    end

    Npaths = size(paths,2);


    if Nfog ~= 0

        fogploss = fspl(path_fog,lambda);
        fogloss = fogpl(path_fog,fc,20,fograte);
        fogdop = 0;
    
        for m = 1:Nfog
    
            rcs = db2pow(value(rcs_fog(m),0,0,fc));
            rgain = aperture2gain(rcs,lambda);
    
    
            % two way
            
            plength = 2 * path_fog(:,m);
            ploss = 2 * fogploss(:,m);
            pfogloss = 2 * fogloss(m,:);
            pdop = fogdop;
            
            paths(Npaths+m).PathLength = plength;
            paths(Npaths+m).PathLoss = ploss + pfogloss;
            paths(Npaths+m).ReflectionCoefficient = db2mag(rgain);
            paths(Npaths+m).AngleOfDeparture = [0;0];
            paths(Npaths+m).AngleOfArrival = [0;0];
            paths(Npaths+m).DopplerShift = pdop;
        end
    end
    
    % sv_tgt = bsxfun(@times,1./sqrt(1),step(txArray,fc,tgt_ang_rel));
    % arrayEffect_tgt = step(rxArray,fc,tgt_ang_rel);
    % 
    % if Nfog ~= 0
    %     rain_fog = zeros(2,Nfog);
    %     sv_fog = bsxfun(@times,1./sqrt(1),step(txArray,fc,rain_fog));
    %     arrayEffect_fog = step(rxArray,fc,rain_fog);
    % else
    %     sv_fog = [];
    %     arrayEffect_fog = [];
    % end
    % 
    % sv = [sv_tgt sv_fog];
    % svv = sum(sv);
    % arrayEffect = [arrayEffect_tgt arrayEffect_fog];

end
