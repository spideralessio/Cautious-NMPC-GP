function [c] = get_c(x,track)
    p = round((x(1:2)-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
    [rows, cols] = size(track.D); % get num rows and cols of distance matrix
    idx = track.idx(rows-p(2), p(1)); % get index of [xc, yc] in track.center_rounded
    xc_i = track.center_rounded(1,idx);
    yc_i = track.center_rounded(2,idx);
    c = [xc_i;yc_i];
end

