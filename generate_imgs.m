clear all
clc
close all

laps = {'noiseLap', 'lrLap'}
% laps = {'noprogressLap', 'baseLap', 'noiseLap', 'lrLap'}
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
    plot(track.outer(1,:),track.outer(2,:), 'black')
    hold on
    plot(track.inner(1,:),track.inner(2,:), 'black')
    % plot(track.center(1,:),track.center(2,:),'--r')
plots = [];

clrs = [
    %0.8 0.2 0;  % base
    %0 0.2 1;   % noprogress
    %0.40 0.63 0.55; % noise
    %0.4 0.7 0.8   % lr
]
colororder(clrs);

for j=1:length(laps)
    lap_name = laps{j};
    load(strcat(lap_name, '.mat'));
    plots = [plots plot(data.x(1,:), data.x(2,:), 'LineWidth', 3)];
end

lgd = legend(plots, laps);
lgd.FontSize = 18;

axis([-1.5,1.9,-2,1.9])
set(gca, "XColor", 'none', 'YColor', 'none')
%saveas(gcf, 'laps.jpg')
%set(gcf, 'Position', [0 0 500, 500]);
%print(gcf, 'laps_20.png', '-dpng', '-r300');

for j=1:length(laps)
    lap_name = laps{j}
    load(strcat(lap_name, '.mat'));
    length(data.x) * 0.03
end
