function [final_cost] = cost_function(X,U,e,data,params)
    
    ModelParams=bycicle_params();
    [N, ~] = size(X);
    final_cost = 0;
    track = params.track;
    trackWidth = params.trackWidth;
    gamma = 0.5;
    last_idx = 0;
    for i=1:N
       state_i = X(i,:)';
       input_i = U(i,:)';
       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       
       
       p = round(([x_i; y_i]-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
       [rows, cols] = size(track.D); % get num rows and cols of distance matrix
%        [x_i, y_i, p(1), p(2)]
       p = max(p,1);
       p = min(p,[cols;rows-1]);
       idx = track.idx(rows-p(2), p(1)); % get index of [xc, yc] in track.center_rounded
       if(last_idx == 0)
           last_idx = idx;
       end
       xc_i = track.center_rounded(1,idx);
       yc_i = track.center_rounded(2,idx);
       if idx == length(track.center)
           phi_i = atan2(track.center_rounded(2,1)- yc_i, track.center_rounded(1,1)- xc_i) - pi;
       else
           phi_i = atan2(track.center_rounded(2,idx+1)- yc_i, track.center_rounded(1,idx+1)- xc_i) - pi;
       end
       
       el_i = -cos(phi_i)*(x_i - xc_i) - sin(phi_i)*(y_i-yc_i);
       ec_i = sin(phi_i)*(x_i - xc_i) - cos(phi_i)*(y_i-yc_i);

%            lreg_i = norm(input_i - U(i-1,:)')^2;

       progress = track.progress(last_idx, idx);
%            if idx > length(track.center)
%                 progress = -progress
%                 idx = mod(idx, track.points)
%            end
%            progress
%            lreg_i
%            5*ec_i^2
%            5*el_i^2
%            idx
%             lreg_i = 1;
       final_cost = final_cost + 5*ec_i^2 + 5*el_i^2 -progress + e;% + lreg_i;
       last_idx = idx;
    end
end

