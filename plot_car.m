function [] = plot_car(X, track)
    ModelParams = bycicle_params();
    hold off
    figure(1);
    plot(track.outer(1,:),track.outer(2,:),'r')
    hold on
    plot(X(:,1), X(:,2), 'b');
    c = [];
    [l, ~] = size(X);
    for j=1:l
       c = [c, get_c(X(j,:)', track)];
    end
    
    scatter(c(1,:), c(2,:))
    plot(track.inner(1,:),track.inner(2,:),'r')
    plot(track.center(1,:),track.center(2,:),'--r')
    carBox(X(1,:)',ModelParams.W/2,ModelParams.L/2)


end