%x = [0.5208830, 0.30461680, 0.296305]';
%x = [0.3739, -0.4064, 0.3662]';
x = [0.517098, -0.128176, 0.329092]';
x_T = [0.3739, -0.4064, 0.3662]';
tol = 0.001;

% dis = norm(x-x_T);
velocity = SEDS(x-x_T,Mu,Sigma)

% while true
% for i=1:10000
%     fprintf("i is %i \n",i);
%     velocity = SEDS(x-x_T,Mu,Sigma);
%     fprintf("velocity is %d, %d, %d \n",velocity(1),velocity(2),velocity(3));
% 
%     if (abs(velocity(1))<=0.001) && (abs(velocity(2))<=0.001) && (abs(velocity(3))<=0.001)
%         disp("tol here");
%         %break;
%     end
% 
%     for j=1:3
%         x(j) = x(j) + velocity(j) * 0.001;
%     end
% 
%     if norm(x-x_T) <= 0.001
%         fprintf("position is %d, %d, %d \n",x(1),x(2),x(3));
%         disp("target found");
%         break;
%     end
% 
%     fprintf("position is %d, %d, %d \n",x(1),x(2),x(3));
%     fprintf("-------------------------- \n");
% 
% end

function y = SEDS(x,Mu,Sigma)
    nbVar = size(Mu,1);
    nbStates = size(Sigma,3);
    in = 1:3;
    out = 4:6;

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
        y = y + beta(j) * yj_tmp;
    end
end


function prob = gaussPDF(Data, Mu, Sigma)
    % Assuming Data, Mu are column vectors and Sigma is a square matrix
    nbVar = numel(Data);
    Data = Data - Mu;
    prob = (Data'/Sigma) * Data;
    prob = exp(-0.5 * prob) / sqrt((2*pi)^nbVar * (abs(det(Sigma)) + realmin));
end
