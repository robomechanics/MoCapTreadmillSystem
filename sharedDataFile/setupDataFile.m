function [memLoc] = setupDataFile(filename,arraySize)
%% USER INPUTS
% setup double[9] array
%arraySize = 9;

%% Setup File
% get file location
fileLoc = fullfile(tempdir, filename);
% Create the communications file if it is not already there.
if ~exist(fileLoc, 'file')
    [f, msg] = fopen(fileLoc, 'wb');
    if f ~= -1
        % setup double[arraySize] array
        fwrite(f, zeros(1,arraySize), 'double');
        fclose(f);
    else
        error('MATLAB:demo:answer:cannotOpenFile', ...
              'Cannot open file "%s": %s.', fileLoc, msg);
    end
end

% Memory map the file.
memLoc = memmapfile(fileLoc, 'Writable', true, 'Format', 'double');