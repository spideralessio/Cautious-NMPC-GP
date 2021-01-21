last_idx = 0;
m = -1.2398;
M = 0.4500;
hold off

xs = [];
ys = [];
cs = [];
for x_i=-1.5:0.01:2
    for y_i=-2:0.01:2
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

       c = el_i + ec_i;
%        if c< m
%            m = c;
%        end
%        
%        if c>M
%            M = c;
%        end
        c = (c-m)/(M-m);
        cs = [cs;c];
        xs = [xs;x_i];
        ys = [ys;y_i];
        
        
    end
end