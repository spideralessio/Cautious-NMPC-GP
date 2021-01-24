function [] = plot_car(X, track)
    ModelParams = bycicle_params();
    hold off
    figure(1);
    plot(track.outer(1,:),track.outer(2,:),'r')
    hold on
    plot(X(:,1), X(:,2), 'b');
    c = [];
    [l, ~] = size(X);
    % for j=1:l
    %    c = [c, get_c(X(j,:)', track)];
    % end
    for j=1:l
        x_i = X(j,ModelParams.stateindex_x);
        y_i = X(j,ModelParams.stateindex_y);
        theta_i = X(j,ModelParams.stateindex_theta);

        [~, idx] = min(abs(track.progress(1,:)-theta_i));

        if (idx > 1)
        tmp_idx = idx-1;
        else
        tmp_idx = idx+1;
        end
        xc_i = ((track.center_rounded(1,idx) - track.center_rounded(1,tmp_idx)) / (track.progress(1,idx) - track.progress(1,tmp_idx))) * (theta_i - track.progress(1,idx)) + track.center_rounded(1,idx); %linear interpolation for xc
        yc_i = ((track.center_rounded(2,idx) - track.center_rounded(2,tmp_idx)) / (track.progress(1,idx) - track.progress(1,tmp_idx))) * (theta_i - track.progress(1,idx)) + track.center_rounded(2,idx); %linear interpolation for yc
        c = [c, [xc_i, yc_i]'];
    end
    
    scatter(c(1,:), c(2,:))
    plot(track.inner(1,:),track.inner(2,:),'r')
    plot(track.center(1,:),track.center(2,:),'--r')
    carBox(X(1,:)',ModelParams.W/2,ModelParams.L/2)


end