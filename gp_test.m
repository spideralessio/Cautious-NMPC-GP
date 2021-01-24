clear all
clc
rng(0,'twister'); % For reproducibility
n = 1000;
x = linspace(-10,10,n)';
y = 1 + x*5e-2 + sin(x)./x + 0.2*randn(n,1);

kfcn = @(XN,XM,theta) (exp(theta(2))^2)*exp(-(pdist2(XN,XM).^2)/(2*exp(theta(1))^2));
sigmaL0 = exp(1.5);
sigmaF0 = exp(0.2);
gprMdl = fitrgp(x,y,'KernelFunction','squaredexponential','KernelParameters',[sigmaL0,sigmaF0]);
L = resubLoss(gprMdl)


scatter(x, y)

xnew= 0
ynew = predict(gprMdl, xnew);
hold on
scatter(xnew, ynew, 'filled')
plot(x, 1 + x*5e-2 + sin(x)./x)