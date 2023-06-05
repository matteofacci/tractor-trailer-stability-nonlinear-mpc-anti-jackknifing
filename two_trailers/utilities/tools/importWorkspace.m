function importWorkspace()
% Prompt user to select directory containing .mat file
dirpath = uigetdir('','Select directory containing .mat file');

% Construct full file path
filename = uigetfile(fullfile(dirpath, '*.mat'), 'Select .mat file');
if isequal(filename, 0) % User cancelled file selection
    disp('File selection cancelled')
    return
end
filepath = fullfile(dirpath, filename);

% Load variables from .mat file into current workspace
%load(filepath);
evalin('base', sprintf('load(''%s'')', filepath));

% % Display confirmation message
% disp('Variables imported from .mat file:')
% who % Display the list of variables in the workspace
end