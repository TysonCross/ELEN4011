%% Main
set(0,'defaultAxesFontName', 'CMU Serif');                  % Make fonts pretty
set(0,'defaultTextFontName', 'CMU Serif');                  % CMU Serif
set(0,'defaultTextFontSize', 12);                           %
set(0,'defaultAxesFontSize', 10);                           % Make fonts readable
set(0,'defaultfigurecolor',[1 1 1]);                        % White BG
set(groot,'FixedWidthFontName', 'ElroNet Monospace')        %
%c = listfonts
tempdir = '/home/tyson/Documents/MATLAB/tmp';
delete(fullfile(tempdir,'*'));
addpath('~/Design','~/Design/matlab');
% addpath('~/Design/matlab/examples/simulinkDroneReferenceApp');