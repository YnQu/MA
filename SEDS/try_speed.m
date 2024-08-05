nbData = size(x,2);
nbVar = 6;
nbStates = size(Sigma,3);
in = 1:3;
out = 4:6;

x = [0.15, 0.66, -0.01]';

for i=1:nbStates
  Pxi(:,i) = gaussPDF(x, Mu(in,i), Sigma(in,in,i));
end
beta = (Pxi./repmat(sum(Pxi,2)+realmin,1,nbStates))';

y = zeros(length(out), nbData);
Sigma_y = zeros(length(out), length(out), nbData);

for j=1:nbStates 
    yj_tmp = Mu(out,j) + Sigma(out,in,j)*inv(Sigma(in,in,j)) * (x(:,i)-Mu(in,j));
    y(:,i) = y(:,i) + beta(j,i).*yj_tmp;
end


function prob = gaussPDF(Data, Mu, Sigma)

    [nbVar,nbData] = size(Data);
    
    Data = Data' - Mu';
    prob = sum((Data/Sigma).*Data, 2);
    prob = exp(-0.5*prob) / sqrt((2*pi)^nbVar * (abs(det(Sigma))+realmin));
end