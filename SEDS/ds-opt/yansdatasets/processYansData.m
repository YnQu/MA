close all; clear all; clc
% Define the filenames
filenames = {'follower_1.txt', 'follower_2.txt', 'follower_3.txt'};

% Initialize a cell array to store the data from each file
data = cell(length(filenames), 1);

% Sampling rate in Hz
samplingRate = 1000; 
timeStep = 1 / samplingRate;

% Loop through each file
for i = 1:length(filenames)
    % Read the data from the file
    T = readtable(filenames{i}, 'Delimiter', '\t');
    
    % Select the columns Xl_x, Xl_y, and Xl_z
    selectedData = T{:, {'Xl_x', 'Xl_y', 'Xl_z'}}';
    
    % Get the number of columns
    N = size(selectedData, 2);
    
    % Create a 6xN matrix, initialize with zeros
    matrix6xN = zeros(6, N);
    
    % Place the selected data into the first 3 rows
    matrix6xN(1:3, :) = selectedData;
    
    % Calculate the velocities using finite differences
    velocities = diff(selectedData, 1, 2) / timeStep;
    
    % Append a zero velocity at the end to match the dimensions
    velocities = [velocities, zeros(3, 1)];
    
    % Place the velocities into the last 3 rows
    matrix6xN(4:6, :) = velocities;
    
    % Store the matrix in the cell array
    data{i} = matrix6xN;
end

% Define the path to save the .mat file
savePath = '/Users/yanqu/Desktop/Thesis/Code_before_lab/icra19-lfd-tutorial-exercises/exercise1_learning/ds-opt/datasets/3D_Yan_test.mat';

% Save the data cell array to the specified path
save(savePath, 'data');

