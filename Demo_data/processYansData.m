close all; clear all; clc

% Define the filenames
nbSamples = 5;
filenames = cell(length(nbSamples),1);
dataset_path = '/Users/yanqu/Desktop/Thesis/MA/Demo_data/';
for n = 1:nbSamples
    filenames{n} = [dataset_path 'follower_' num2str(n) '.txt'];
end

% Initialize a cell array to store the data from each file
data = cell(length(filenames), 1);

% Loop through each file
for i = 1:length(filenames)
    % Read the data from the file
    T = readtable(filenames{i}, 'Delimiter', '\t');

    % Select the columns Xl_x, Xl_y, and Xl_z
    selectedData = T{:, {'Xl_x', 'Xl_y', 'Xl_z', 'Vl_x', 'Vl_y', 'Vl_z'}}';
    selectedData = selectedData(:,2000:end-1000);

    % Store the matrix in the cell array
    data{i} = selectedData;
end

% Define the path to save the .mat file
savePath = '/Users/yanqu/Desktop/Thesis/Code_before_lab/icra19-lfd-tutorial-exercises/exercise1_learning/ds-opt/datasets/3D_Yan_test.mat';

% Save the data cell array to the specified path
save(savePath, 'data');
