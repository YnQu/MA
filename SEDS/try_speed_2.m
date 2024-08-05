nbVar = 6;
nbStates = size(Sigma,3);
in = 1:3;
out = 4:6;

x = [0.5208830, 0.30461680, 0.296305]';

Pxi = zeros(1, nbStates);
beta = zeros(nbStates, 1);
y = zeros(length(out), 1);
Sigma_y = zeros(length(out), length(out));

% Compute the Gaussian PDFs for each state for the input
for i=1:nbStates
  Pxi(i) = gaussPDF(x, Mu(in,i), Sigma(in,in,i));
end

% Compute the responsibilities (beta)
beta = Pxi / (sum(Pxi) + realmin);

% Estimate the output (y)
for j=1:nbStates
    yj_tmp = Mu(out,j) + Sigma(out,in,j) * inv(Sigma(in,in,j)) * (x - Mu(in,j));
    y = y + beta(j) * yj_tmp
end


function prob = gaussPDF(Data, Mu, Sigma)
    % Assuming Data, Mu are column vectors and Sigma is a square matrix
    nbVar = numel(Data);
    Data = Data - Mu
    prob = (Data'/Sigma) * Data
    prob = exp(-0.5 * prob) / sqrt((2*pi)^nbVar * (abs(det(Sigma)) + realmin));
end
