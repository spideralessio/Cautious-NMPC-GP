function [cineq] = constraints(X,U,e,data,params)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
    ModelParams=bycicle_params();
    [N, ~] = size(X);
    trackWidth = params.trackWidth;
    track = params.track;
    cineq = [];
    for i=1:N
       state_i = X(i,:)';
       input_i = U(i,:)';

       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       
%        p = round(([x_i; y_i]-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
%        [rows, cols] = size(track.D); % get num rows and cols of distance matrix
%        p = max(p,1);
%        p = min(p,[cols;rows-1]);
%        err = track.D(rows-p(2), p(1)); % evaluate distance from [xc, yc]
%        err = double(err);  
       
       c = get_c([x_i;y_i], track);
       err = norm([x_i;y_i]-c);
       
       cineq = [cineq; err - trackWidth/2;];
       if i>1
            err = norm([x_i;y_i]-c);
            cineq = [cineq; err - trackWidth/2;];
       end
       last_c = c;
    end
    
    
    
end

