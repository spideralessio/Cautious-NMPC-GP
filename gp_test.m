clear all
clc
fn = @(n) 4*n+5  % retta y=4x+5

x_min = 0.5
x_max = 5
step = 0.1

X = [x_min:step:x_max]

y = []
for i=1:length(X)
   y = [y fn(X(i))+0.5*(rand()-0.5)];
end

scatter(X, y)

model = fitrgp(X',y');

xnew = 10
ypred = predict(model, xnew)

hold on
scatter(xnew, ypred, 'filled')