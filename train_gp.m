clear all
clc
close all


load('gp_data.mat');

Bd = [zeros(3); eye(3); zeros(1,3)];
Bd_pinv = pinv(Bd);

X = [data.x(:,1:length(data.x)-1);data.u(:,1:length(data.x)-1)]';

y = data.x(:,2:length(data.x));
size(y)

f = zeros(size(y));

Ts = 0.03
params = struct;
params.modified = true;

for i=1:length(f)
    f(:,i) = Ts*bycicle_model(data.x(:,i), data.u(:,i), params);
end

y = Bd_pinv*(y - f);
 
y = y';

kfcn = @(XN,XM,theta) (theta(2)^2)*exp(-(pdist2(XN,XM).^2)/(2*theta(1)^2));
kfcn_prime = @(XN, XM, theta) ((theta(2)/theta(1))^2)*(theta(1)^2 - pdist2(XN, XM).^2)*exp(-(pdist2(XN,XM).^2)/(2*theta(1)^2))


sigmaL0 = exp(1);
sigmaF0 = exp(0.2);
gprMdl = fitrgp(X,y(:,1),'KernelFunction',kfcn_prime,'KernelParameters',[sigmaL0,sigmaF0]);
L = resubLoss(gprMdl)
modelX = fitrgp(X, y(:,1));
modelY = fitrgp(X, y(:,2));
modelPhi = fitrgp(X, y(:,3));

