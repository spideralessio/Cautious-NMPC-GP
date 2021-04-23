function [c] = get_centers(X, track)
    [rows, cols] = size(track.D); % get num rows and cols of distance matrix
    N = length(X);
    c = zeros(2, N);
    m = [track.xmin; track.ymin];
    step = track.step;

    for i = 1:N
        x = X(i, 1:2)';
        p = round((x-m)/step); 

        % get index of [xc, yc] in track.center_rounded
        idx = track.idx(rows-p(2), p(1)); 
        xc_i = track.center_rounded(1,idx);
        yc_i = track.center_rounded(2,idx);
        c(:, i) = [xc_i;yc_i];
    end
end

