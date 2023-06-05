function saveWorkspace(varnames)
% Prompts user to select a folder and saves the current workspace with a
% filename based on a merge of specified variable values.
% Check if variable names were specified
if nargin < 1
    error('Variable names must be specified');
end

% Prompt user to select a folder to save the workspace
foldername = uigetdir('','Select folder to save workspace');

% Check if user cancelled folder selection
if isequal(foldername,0)
    disp('Folder selection cancelled, workspace not saved.');
    return
end

% Merge variable values as strings using VARNAMES as a guide
filename = '';
for i = 1:length(varnames)
    % Check if variable exists in workspace
%     if ~exist(varnames{i},'var')
%         disp('entro')
%         break
%     end

    % Get variable value and convert to string
    value = evalin('base', varnames{i});
    if isstring(value)
        str_value = value;
    elseif isnumeric(value)
        str_value = num2str(value);
    else
        str_value = '';
    end
    if i == length(varnames)
    % Add variable name and value to filename
        filename = strcat(filename, varnames{i}, '_', str_value);
    else
       filename = strcat(filename, varnames{i}, '_', str_value, '_'); 
    end
end

% Save workspace with merged filename in selected folder
filename = strcat(filename, '.mat');
fullpath = fullfile(foldername, filename);
evalin('base', sprintf('save(''%s'')', fullpath));
disp(['Workspace saved as ' filename ' in ' foldername]);
end