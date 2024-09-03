clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2023
% Basak Gulecyuz
% Chair of Media Technology, TUM
% Learning from demonstration using GMM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% LfD Parameter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 10;                        % Number of GMM states
model.nbD = 100;                            % Number of datapoints
model.nbSamples = 5;                        % Number of demonstrations
%dataset_path = 'TeleopData/';
dataset_path = '/Users/yanqu/Desktop/Thesis/MA/Demo_data/';
%% Load Demonstrations

% Temporal data as Data input for GMR
Data_t = 1:model.nbD;

% Initialize Data for learning
Data = [];
for n=1:model.nbSamples
    
    % Load each demonstration's trajectories
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    demos(n).pos = loadDemos(fileName)';
    demos(n).time = size(demos(n).pos,2);
    
    % Interpolate trajectories to model.nbD points
    demos(n).pos  = spline(1:size(demos(n).pos,2), demos(n).pos, linspace(1,size(demos(n).pos,2),model.nbD)); %Resampling
    
    % Concatenate the demonstrations and time
    Data = [Data; [Data_t' demos(n).pos']];
end


%% Visualize Demonstrations

figure
for n=1:model.nbSamples
    plot3(demos(n).pos(1,:),demos(n).pos(2,:),demos(n).pos(3,:), 'LineWidth', 2.0);
    grid on;
    hold on;
end
legend('demo 1', 'demo 2', 'demo 3');
xlabel('x')
ylabel('y')
zlabel('z')
%% Learning GMM

% To-Do: Fit data to a GMM with nbStates (Hint: see fitgmdist())
% Our goal is to model P(x,t) using GMM
% Using a regularization value helps improve learning (e.g 0.0001)

GMModel = fitgmdist(Data,model.nbStates,'RegularizationValue', 0.0001);
% fitgmdist: Fit a Gaussian mixture distribution to data.

%% Plot Model
figure
for n=1:model.nbSamples
    plot3(demos(n).pos(1,:),demos(n).pos(2,:),demos(n).pos(3,:), 'LineWidth', 2.0);
    grid on;
    hold on;
end
GMM3D_plot(GMModel.mu(:,2:4)', GMModel.Sigma(2:4,2:4,:),1);
xlabel('x')
ylabel('y')
zlabel('z')
%% Reproduction using GMR

% To-Do: Use GMR to obtain the conditional dist. P(x|t)
% We will use the E[P(x|t)] for every t in  Data_t as the trajectory to be
% reproduced. See lecture slides for GMR. Please follow the guideline step
% by step to complete the task.

% 1. Compute the influence of each GMM component, given input t

for i=1:model.nbStates
    
    % Emission probability for state i
    pd_i = makedist('Normal', GMModel.mu(i,1), GMModel.Sigma(1,1,i));
    % makedist is "make a probability distribution", here is making a
    % Normal distribution
    
    % To-Do: compute h_i(data_t,i) (see slide 17.) (Hint: pdf() is helpful)
    h_i(:,i) = GMModel.ComponentProportion(i).*pdf(pd_i, Data_t);
    
end

% Normalize h_i so that sum over states makes 1.
beta = h_i./repmat(sum(h_i,2)+realmin,1,model.nbStates);

% 2. Compute expected means x, given input t

for j=1:model.nbStates
    
    % To-Do: Compute mu_hat_i_O (see slide 17.)
    mu_hat_i_o(:,:,j) = repmat(GMModel.mu(j,2:4)',1,model.nbD) + GMModel.Sigma(2:4,1,j)*inv(GMModel.Sigma(1,1,j))...
        * (Data_t-repmat(GMModel.mu(j,1)',1,model.nbD));
end
beta_tmp = reshape(beta,[1 size(beta)]);
x_tmp = repmat(beta_tmp,[3 1 1]) .* mu_hat_i_o;
x = sum(x_tmp,3);


%% Plot Reproductions
figure
for n=1:model.nbSamples
    plot3(demos(n).pos(1,:),demos(n).pos(2,:),demos(n).pos(3,:), 'LineWidth', 2.0);
    grid on;
    hold on;  
end
plot3(x(1,:),x(2,:),x(3,:),'LineWidth', 2.0);
legend('demo 1', 'demo 2', 'demo 3', 'reproduction');


% Interpolate the reproduction and save

x = spline(1:size(x,2), x, linspace(1,size(x,2),model.nbD*120)); %Resampling
fileName = [dataset_path 'learned_.csv'];
csvwrite(fileName,x);


