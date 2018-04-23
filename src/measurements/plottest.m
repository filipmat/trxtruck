% Experimenting with plot looks
% https://se.mathworks.com/help/matlab/ref/axes-properties.html
% https://se.mathworks.com/help/matlab/ref/figure-properties.html
% https://se.mathworks.com/help/matlab/ref/plot.html#inputarg_LineSpec

%%
load('c_vbrake_d03_v4.mat')
c = cells;

plothandle = figure;
hold on
lgd = cell(3*2, 1);
for i = 1:3
    plot(c{i}.t, c{i}.v)
    lgd{i} = c{i}.id;
end

ax = gca;
ax.ColorOrderIndex = 1;
for i = 1:3
    plot(c{i}.t, c{i}.vref, '--')
    lgd{3 + i} = c{i}.id + " reference";
end

legend(lgd, 'Location', 'southeast')

%% Set title or labels.
titletext = 'Simulation';
xlabeltext = 'Time (s)';
ylabeltext = 'Speed (m/s)';

title(titletext)
xlabel(xlabeltext)
ylabel(ylabeltext)

%% Specify and set properties.

% Specify properties. 
width = 600;                % Width of figure window (default in pos(3))
height = 400;               % Height of figure window (default in pos(4))
axislinewidth = 1.2;        % Axis line width
linewidth = 1.2;            % Plot line with
markersize = 15;            % MarkerSize
plotcolor = [0, 0, 0];

axisfontsize = 13;          % Font size of axis ticks
labelfontsize = 15;         % Font size of x- and y-label
titlefontsize = 16;         % Font size of title
titlefontweight = 'normal';
mainfont = 'Serif';     % For available fonts, type listfonts.
axisfont = mainfont;        % (default: Helvetica)
labelfont = mainfont;       % (default: Helvetica)

xlimits = [0, 29]; % get(gca, 'XLim'); % X-axis limits [xmin, xmax]
ylimits = [0, 1.4];           % Y-axis limits (default: get(gca, 'YLim'))
xtick = get(gca, 'XTick');  % X-axis ticks [x1, x2, x3, ..., xn].
ytick = get(gca, 'YTick');       % Y-axis ticks (default: get(gca, 'YTicks'))

pos = get(gcf, 'Position');

% Set Properties.
set(findall(gca, 'Type', 'Line'), 'LineWidth', linewidth);

ax = gca;

ax.Title.FontSize = titlefontsize;
ax.Title.FontName = mainfont;
ax.Title.FontWeight = titlefontweight;

ax.XLabel.FontSize = labelfontsize;
ax.XLabel.FontName = labelfont;

ax.YLabel.FontSize = labelfontsize;
ax.YLabel.FontName = labelfont;

% Set axis properties.
set(gca,...
    'XLim', xlimits, ...            % Axis limits
    'YLim', ylimits, ...            
    'FontSize', axisfontsize, ...
    'LineWidth', axislinewidth, ...
    'FontName', axisfont, ...
    'XTick', xtick, ...             % Which tick-numbers that appear
    'YTick', ytick, ...
    'XMinorTick', 'on', ...         % Minor tickmarks inbetween
    'YMinorTick', 'on', ...
    'Box', 'off', ...               % Box around figure
    'TickDir', 'out', ...           % Tickmarks outside or inside of axis
    'XGrid', 'off', ...             % Background grid
    'YGrid', 'off');

% Set figure properties.
set(gcf, ...
    'Position', [pos(1), pos(2), width, height], ...    % Set position, size.    
    'Resize', 'off');               % Forbid resize

