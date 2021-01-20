clear all
close all
clc

load 'track2.mat'

track = track2

xs = [track.inner(1,:),track.center(1,:),track.outer(1,:)];
ys = [track.inner(2,:),track.center(2,:),track.outer(2,:)];


xmin = min(xs);
ymin = min(ys);
xmax = max(xs);
ymax = max(ys);

padding = 0.2;

xmin = xmin - padding
xmax = xmax + padding
ymin = ymin - padding
ymax = ymax + padding

xmin = -1.5
xmax = 1.8
ymin = -2
ymax = 1.8



hold on

scatter(xmin, ymin)
scatter(xmax, ymax)
plot(track.inner(1,:),track.inner(2,:))
plot(track.center(1,:),track.center(2,:))
plot(track.outer(1,:),track.outer(2,:))

round_n = 2
step = 10^(-round_n)
rows = (ymax-ymin)/step;
cols = (xmax-xmin)/step;

mat = zeros(rows, cols);
mat_idx = zeros(rows, cols);
points = length(track.center);

track.center_rounded = []
for i=1:length(track.center)
   p = round(track.center(:,i),round_n);
   track.center_rounded = [track.center_rounded, p];
   p = round((p-[xmin;ymin])/step);
   mat(rows-p(2), p(1)) = 1;
   mat_idx(rows-p(2), p(1)) = i;
end

assert(points == length(track.center_rounded))


progress = zeros(points, points);

for i=2:points
    d = norm(track.center_rounded(:,i) - track.center_rounded(:,i-1));
    for j=1:i-1
        p = progress(j,i-1) + d;
        progress(j, i) = p;
        progress(i, j) = -p;
    end
end

[D,idx] = bwdist(mat);

for r=1:rows
    for c=1:cols
        idx(r,c) = mat_idx(idx(r,c));
    end
end

D = D*step;

track.D = D;
track.idx = idx;
track.progress = progress;
track.round_n = round_n;
track.step = step;
track.xmin = xmin;
track.ymin = ymin;
save("track.mat", "track")