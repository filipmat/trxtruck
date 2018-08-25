% Experimenting with plot looks
% https://se.mathworks.com/help/matlab/ref/axes-properties.html
% https://se.mathworks.com/help/matlab/ref/figure-properties.html
% https://se.mathworks.com/help/matlab/ref/plot.html#inputarg_LineSpec

%% Set title or labels.
titletext = '';
xlabeltext = 'time [s]';
ylabeltext = 'mean absolute error [m]';

title(titletext)
xlabel(xlabeltext)
ylabel(ylabeltext)

%% Specify and set properties.

% Specify properties. 
width = 600;                % Width of figure window (default in pos(3))
height = 400;               % Height of figure window (default in pos(4))
axislinewidth = 1.2;        % Axis line width
linewidth = 2;            % Plot line with
markersize = 15;            % MarkerSize
plotcolor = [0, 0, 0];

axisfontsize = 13;          % Font size of axis ticks
labelfontsize = 15;         % Font size of x- and y-label
titlefontsize = 16;         % Font size of title
titlefontweight = 'normal';
mainfont = 'Serif';     % For available fonts, type listfonts.
axisfont = mainfont;        % (default: Helvetica)
labelfont = mainfont;       % (default: Helvetica)

xlimits = [0, 60]; % get(gca, 'XLim'); % X-axis limits [xmin, xmax]
ylimits = [0.0, 0.15];           % Y-axis limits (default: get(gca, 'YLim'))
xtick = 0:10:60;
%xtick = get(gca, 'XTick');  % X-axis ticks [x1, x2, x3, ..., xn].
ytick = 0:0.025:1.6;
%ytick = get(gca, 'YTick');       % Y-axis ticks (default: get(gca, 'YTicks'))

legendlocation = 'southeast';

pos = get(gcf, 'Position');

change_colors = 1;
change_linestyles = 1;

draw_grid = true;

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

ax.Legend.Location = legendlocation;

% Set axis properties.
set(gca,...
    'XLim', xlimits, ...            % Axis limits
    'YLim', ylimits, ...            
    'FontSize', axisfontsize, ...
    'LineWidth', axislinewidth, ...
    'FontName', axisfont, ...
    'XTick', xtick, ...             % Which tick-numbers that appear
    'YTick', ytick, ...
    'XMinorTick', 'off', ...         % Minor tickmarks inbetween
    'YMinorTick', 'off', ...
    'Box', 'off', ...               % Box around figure
    'TickDir', 'out', ...           % Tickmarks outside or inside of axis
    'XGrid', 'off', ...             % Background grid
    'YGrid', 'off');

% Set figure properties.
set(gcf, ...
    'Position', [pos(1), pos(2), width, height], ...    % Set position, size.    
    'Resize', 'off');               % Forbid resize

colors1 = [230, 97, 1; 253, 184, 99; 178, 171, 210; 94, 60, 153]/255;
colors2 = [215, 25, 28; 253, 174, 97; 171, 217, 233; 44, 123, 182]/255;
colors3 = [208, 28, 139; 241, 182, 218; 184, 225, 134; 77, 172, 38]/255;
colors4 = [166, 206, 227; 31, 120, 180; 251, 154, 153; 51, 160, 44; ...
    178, 223, 138]/255;
colors5 = [172, 151, 62; 129, 118, 204; 91, 169, 102; 199, 90, 147; ...
    204, 95, 67]/255;
colors6 = [1, 118, 21; 255, 163, 236; 206, 170, 26; 162, 0, 70; ...
    135, 144, 71]/255;
colors7 = [230, 87, 60; 1, 95, 193; 132, 168, 21; 199, 24, 122; ...
    0, 164, 86]/255;
colors8 = [68, 119, 170; 238, 102, 119; 34, 136, 51; 204, 187, 68]/255;

linestyles = ["-", "--", ":", "-."];
%linestyles = ["-"];

if change_colors
    lines = findobj(ax, 'Type', 'line');
    ll = length(lines);
    % colors = distinguishable_colors(ll);
    % colors = linspecer(ll);
    colors = colors8;
    for i = fliplr(1:ll)
        j = ll + 1 - i;
        lines(j).Color = colors(mod(i - 1, length(colors)) + 1, :);
        if change_linestyles
            lines(j).LineStyle = linestyles(mod(i - 1, length(linestyles)) + 1);
        end
    end
end

if draw_grid
    grid on
end
