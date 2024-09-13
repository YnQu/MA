close all; clear all; clc

% Define the filenames
nbSamples = 5;
filenames = cell(length(nbSamples),1);
% Mac:
%dataset_path = '/Users/yanqu/Desktop/Thesis/MA/Demo_data/';

% Ubuntu:
dataset_path = '/home/yan/MA/Peeling_like/State_1/';
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
    figure(1)
    plot(selectedData(4,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
    plot(selectedData(5,:)', '.-','Color',[1 0 0], 'LineWidth',1); hold on;
    plot(selectedData(6,:)', '.-','Color',[0 1 0], 'LineWidth',1); hold on;
    selectedData = selectedData(:,1200:end-2000);
    figure(2);
    plot(selectedData(4,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
    plot(selectedData(5,:)', '.-','Color',[1 0 0], 'LineWidth',1); hold on;
    plot(selectedData(6,:)', '.-','Color',[0 1 0], 'LineWidth',1); hold on;

    % Store the matrix in the cell array
    data{i} = selectedData;
end

% Define the path to save the .mat file
% savePath = '/Users/yanqu/Desktop/Thesis/MA/SEDS/ds-opt/datasets/3D_Yan_test.mat';
savePath = '/home/yan/MA/Peeling_like/datasets/State_1.mat';
% Save the data cell array to the specified path
save(savePath, 'data');

