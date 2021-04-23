clear all
clc
close all

laps = {'noiseLap', 'lrLap'}
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

clrs = [
    0.8 0.7 0;
    0.9 0.3 0;
    0 0.2 1;
    0 0.5 0.5
]
colororder(clrs);

for j=1:length(laps)
    lap_name = laps{j};
    load(strcat(lap_name, '.mat'));
    plots = [plots plot(data.x(1,:), data.x(2,:), 'LineWidth', 3)];
end

lgd = legend(plots, laps);
lgd.FontSize = 16;

axis([-1.5,2.7,-2,2.5])
%saveas(gcf, 'laps.jpg')
%set(gcf, 'Position', [0 0 500, 500]);
%print(gcf, 'laps_20.png', '-dpng', '-r300');

for j=1:length(laps)
    lap_name = laps{j}
    load(strcat(lap_name, '.mat'));
    length(data.x) * 0.03
end
