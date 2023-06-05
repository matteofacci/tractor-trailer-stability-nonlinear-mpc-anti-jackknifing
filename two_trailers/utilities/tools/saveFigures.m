function saveFigures(format)

% PDF: pdf
% EPS: eps
% JPEG: jpeg
% TIFF: tiff
% PNG: png

folder = uigetdir();
figHandles = findobj(0, 'Type', 'figure');
for i = 1:length(figHandles)
    name = get(figHandles(i), 'Name');
    if isempty(name)
        name = ['figure' num2str(i)];
    end
    %print(figHandles(i), fullfile(folder, [name]), format)
    filename = [name, '.', format];
    fullfilepath = fullfile(folder, filename);
    exportgraphics(figHandles(i), fullfilepath, 'Resolution', 300,'BackgroundColor','none');
end