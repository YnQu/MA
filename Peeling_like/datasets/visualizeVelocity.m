close all; clear all; clc

% Define the filenames
nbTest = 3;
filenames = cell(length(nbTest),1);
refenames = cell(length(nbTest),1);
%dataset_path = '/Users/yanqu/Desktop/Thesis/MA/Demo_data/';
dataset_path = '/home/yan/MA/Peeling_like/datasets/';
for n = 1:nbTest
    filenames{n} = [dataset_path 'Test_' num2str(n) '.txt'];
    refenames{n} = [dataset_path 'State_' num2str(n) '.mat'];
end


% Initialize a cell array to store the data from each file
data = cell(length(filenames), 1);

% Loop through each file
for i = 1:length(filenames)
    % Read the data from the file
    T = readtable(filenames{i}, 'Delimiter', '\t');
    Ref = load(refenames{i});

    % Select the columns Xl_x, Xl_y, and Xl_z
    selectedData = T{:, {'Xl_x', 'Xl_y', 'Xl_z', 'Vl_x', 'Vl_y', 'Vl_z'}}';
    %selectedData = selectedData(:,1:10:end);
    figure(i);
    plot(selectedData(4,:)', '.-','Color',[0 0 1], 'LineWidth',0.5); hold on;
    plot(selectedData(5,:)', '.-','Color',[1 0 0], 'LineWidth',0.5); hold on;
    plot(selectedData(6,:)', '.-','Color',[0 1 0], 'LineWidth',0.5); hold on;

    for j = 1:5
        refData = Ref(1);
        plot(refData(4,:)', '--','Color',[0 0 1], 'LineWidth',0.5); hold on;
        plot(refData(5,:)', '--','Color',[1 0 0], 'LineWidth',0.5); hold on;
        plot(refData(6,:)', '--','Color',[0 1 0], 'LineWidth',0.5); hold on;

    end

    plot(State_1())

    % Store the matrix in the cell array
    data{i} = selectedData;
end

%% Define the path to save the .mat file
%savePath = '/Users/yanqu/Desktop/Thesis/MA/SEDS/ds-opt/datasets/3D_Yan_test.mat';
%savePath = '/home/yan/MA/Demo_data/State_2.mat';
%savePath = '/home/yan/MA/Peeling_like/datasets/State_2.mat';

%% Save the data cell array to the specified path
%save(savePath, 'data');

