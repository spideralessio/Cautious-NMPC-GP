clear all
clc
close all

laps = {'noprogress_lap', 'base_lap', 'noise_lap', 'gp_lap'}
load('track.mat');

% for j=1:length(laps)
%     lap_name = laps{j};
%     lap = 1;
%     t = 0;
%     Ts = 0.03;
%     h = figure
%     filename = strcat(lap_name, '.gif');
%     load(strcat(lap_name, '.mat'));
%     for i=1:length(data.x)
%         plot_car(data.x(:,i)', track, t, lap);
%         %pause(0.01)
%         frame = getframe(h);
%         im = frame2im(frame);
%         [imind, cm] = rgb2ind(im, 256);
%         if t == 0
%             imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.03);
%         else
%             imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.0);
%         end
% 
%         t = t + Ts
%     end
% end



close all
hold off
    figure(1);
    plot(track.outer(1,:),track.outer(2,:),'r')
    hold on
    plot(track.inner(1,:),track.inner(2,:),'r')
    plot(track.center(1,:),track.center(2,:),'--r')
plots = [];
for j=1:length(laps)
    lap_name = laps{j};
    load(strcat(lap_name, '.mat'));
    plots = [plots plot(data.x(1,:), data.x(2,:))];
end
legend(plots, laps);
axis([-1.5,2.7,-2,2.5])