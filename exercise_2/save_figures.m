% Script to save all current open figures

% Create a folder for plots if it doesn't exist
if ~exist('plots', 'dir')
    mkdir('plots');
end

% Get all open figures
figHandles = findall(0, 'Type', 'figure');

for i = 1:length(figHandles)
    f = figHandles(i);
    
    % Get the name we assigned to the figure
    name = get(f, 'Name');
    
    % Clean up the name for a filename (remove spaces/special chars)
    if isempty(name)
        filename = sprintf('Figure_%d', f.Number);
    else
        % Remove "Task #: " if present, replace spaces with underscores
        cleanName = regexprep(name, '[^a-zA-Z0-9_]', '_');
        filename = cleanName;
    end
    
    fullPath = fullfile('plots', [filename '.pdf']);
    
    fprintf('Saving %s ...\n', fullPath);
    
    % Export vector images
    exportgraphics(f, fullPath, 'ContentType', 'vector'); 
end

fprintf('All plots saved to the /plots folder.\n');