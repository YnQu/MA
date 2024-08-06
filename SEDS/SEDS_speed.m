x = [0.5208830, 0.30461680, 0.296305]';
%x = [0.3690, -0.4051, 0.3602]';
x_T = [0.3739, -0.4064, 0.3662]';
tol = 0.001;

dis = norm(x-x_T);

%while true
for i=1:10000
    fprintf("i is %i \n",i);
    velocity = SEDS(x,Priors,Mu,Sigma);
    fprintf("velocity is %d,%d,%d \n",velocity(1),velocity(2),velocity(3));
   
    if (abs(velocity(1))<=0.001) && (abs(velocity(2))<=0.001) && (abs(velocity(3))<=0.001)
    %if i == 10
        disp("tol here");
        break;
    end
  
    for j=1:3
        x(j) = x(j) + velocity(j) * 0.001;
    end

    if norm(x-x_T) <= 0.01
        disp("target found");
        break;
    end
    %x = x_d;
    
    %fprintf("position desired is %d, %d, %d \n",x_d(1),x_d(2),x_d(3));
    fprintf("position is %d,%d,%d \n",x(1),x(2),x(3));
    fprintf("-------------------------- \n");
    
end


function y = SEDS(x,Priors,Mu,Sigma)
    nbVar = size(Mu,1);
    nbStates = size(Sigma,3);
    in = 1:3;
    out = 4:6;
    nbData = size(x,2);
    
    for i=1:nbStates
      Pxi(:,i) = Priors(i).*gaussPDF(x, Mu(in,i), Sigma(in,in,i));
    end
    beta = Pxi./repmat(sum(Pxi,2)+realmin,1,nbStates);
    
    for j=1:nbStates
      y_tmp(:,:,j) = repmat(Mu(out,j),1,nbData) + Sigma(out,in,j)/(Sigma(in,in,j)) * (x-repmat(Mu(in,j),1,nbData));
    end
    beta_tmp = reshape(beta,[1 size(beta)]);
    y_tmp2 = repmat(beta_tmp,[length(out) 1 1]) .* y_tmp;
    y = sum(y_tmp2,3);
end