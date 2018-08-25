load('cmpc_v1_trx_v2_00.mat')
%load('c_normal_d00.mat')
c = cells;
c_descr = descr;
load('dmpc_v1_trx_v2_01.mat')
%load('d_normal_d00.mat')
d = cells;
d_descr = descr;

%% speed centralized
lc = length(c);
figure;
hold on
lgd = cell(lc, 1);
for i = 1:lc
    plot(c{i}.pos, c{i}.v)
    lgd{i} = c{i}.id;
end
plot(c{3}.pos, c{3}.vref)
lgd = [lgd; 'reference'];
legend(lgd)

%% speed distributed
ld = length(d);
figure;
hold on
lgd = cell(ld, 1);
for i = 1:ld
    plot(d{i}.pos, d{i}.v)
    lgd{i} = d{i}.id;
end
plot(d{1}.pos, d{1}.vref)
lgd = [lgd; 'reference'];
legend(lgd)

%% braking centralized
lc = length(c);
figure;
hold on
lgd = cell(lc, 1);
obst = c{1}.pos(200) + 5;

for i = 1:lc
    %plot(c{i}.t, c{i}.pos + 20 - c{1}.pos(200))
    plot(c{i}.t - c{i}.t(1), c{i}.pos);
    lgd{i} = c{i}.id;
end
%plot([0, 30], [25, 25])
%plot([c{1}.t(1), c{1}.t(end)], [44.44, 44.44])
plot([0, c{1}.t(end) - c{1}.t(1)], [obst, obst])
lgd = [lgd; 'obstacle'];
legend(lgd)

%% braking distributed

% distributed c{1}.pos(201) + 5: obstacle braking.
% centralized pos(200) + 5

ld = length(d);
figure;
hold on
lgd = cell(ld, 1);
for i = 1:ld
    plot(d{i}.t, d{i}.pos + 20 - d{1}.pos(201))
    lgd{i} = d{i}.id;
end
plot([0, 30], [25, 25])
lgd = [lgd; 'obstacle'];
legend(lgd)

%% path centralized
lc = 1;%length(c);
r = 1:150;
figure;
hold on
lgd = cell(lc, 1);
for i = 2:2
    plot(c{i}.x(r), c{i}.y(r))
end

xx = c_descr.xc + c_descr.xr*cos(0:0.05:(2*pi + 0.1));
yy = c_descr.yr*sin(0:0.05:(2*pi + 0.1));

plot(xx, yy)

lgd = {'vehicle trajectory', 'reference'};

legend(lgd)

%% timegap centralized
lc = length(c);
figure;
hold on
lgd = cell(lc, 1);
for i = 1:lc
    plot(c{i}.t - c{1}.t(1), c{i}.timegap)
    lgd{i} = c{i}.id;
end
legend(lgd)
plot(c{1}.t - c{1}.t(1), ones(length(c{1}.t), 1));
lgd = [lgd; 'reference'];
legend(lgd)

%% timegap distributed
ld = length(d);
figure;
hold on
lgd = cell(ld, 1);
for i = 1:ld
    plot(d{i}.t - d{1}.t(1), d{i}.timegap)
    lgd{i} = d{i}.id;
end
legend(lgd)
plot(d{1}.t - d{1}.t(1), ones(length(d{1}.t), 1));
lgd = [lgd; 'reference'];
legend(lgd)

%% acceleration centralized
lc = length(c);
figure;
hold on
lgd = cell(lc, 1);

for i = 1:lc
    plot(c{i}.t - c{i}.t(1), c{i}.acc);
    lgd{i} = "a_" + i + "(t)";
end
legend(lgd)

%% acceleration distributed
ld = length(d);
figure;
hold on
lgd = cell(ld, 1);

for i = 1:ld
    plot(d{i}.t - d{i}.t(1), d{i}.acc);
    lgd{i} = "a_" + i + "(t)";
end
legend(lgd)

%% path error centralized
lc = length(c);
figure;
hold on
lgd = cell(lc, 1);
for i = 1:lc
    plot(c{i}.t - c{i}.t(1), c{i}.path_error)
    lgd{i} = c{i}.id;
end
legend(lgd)

%% path error both
lc = length(c);
ld = length(d);
figure;
hold on

ec = zeros(size(c{1}.t));
ed = zeros(size(d{1}.t));

for i = 1:lc
    ec = ec + [abs(c{i}.path_error); zeros(length(ec) - length(c{i}.t), 1)];
end
ec = ec/lc;

for i = 1:ld
    ed = ed + [abs(d{i}.path_error); zeros(length(ed) - length(d{i}.t), 1)];
end
ed = ed/ld;

plot(c{1}.t - c{1}.t(1), ec, d{1}.t - d{1}.t(1), ed)

lgd = {'centralized', 'distributed'};
legend(lgd)

%% speed error centralized
lc = length(c);
figure;
hold on
lgd = cell(lc, 1);
for i = 1:lc
    plot(c{i}.pos, c{i}.vref - c{i}.v)
    lgd{i} = c{i}.id;
end
legend(lgd)

%% speed error centralized
ld = length(d);
figure;
hold on
lgd = cell(ld, 1);
for i = 1:ld
    plot(d{i}.pos, d{i}.vref - d{i}.v)
    lgd{i} = d{i}.id;
end
legend(lgd)

%% speed error both
lc = length(c);
ld = length(d);
figure;
hold on

ec = zeros(size(c{1}.t));
ed = zeros(size(d{1}.t));

for i = 1:lc
    %ec = ec + sqrt((c{i}.v - c{i}.vref).^2);
    ec = ec + [abs(c{i}.v - c{i}.vref); zeros(length(ec) - length(c{i}.v), 1)];
end
ec = ec/lc;

for i = 1:ld
    %ed = ed + sqrt((d{i}.v - d{i}.vref).^2);
    ed = ed + [abs(d{i}.v - d{i}.vref); zeros(length(ed) - length(d{i}.v), 1)];
end
ed = ed/ld;

plot(c{1}.t - c{1}.t(1), ec)
plot(d{1}.t - d{1}.t(1), ed)

lgd = {'centralized', 'distributed'};
legend(lgd)

%% timegap error both
lc = length(c);
ld = length(d);
figure;
hold on

r = 1:600;

ec = zeros(size(c{1}.t(r)));
ed = zeros(size(d{1}.t(r)));

if lc > 1
    for i = 2:lc
        ec = ec + abs(c{i}.timegap(r) - 1);
    end
    ec = ec/(lc - 1);
end

if ld > 1
    for i = 2:ld
        ed = ed + abs(d{i}.timegap(r) - 1);
    end
    ed = ed/(ld - 1);
end

plot(c{1}.t(r) - c{1}.t(r(1)), ec)
plot(d{1}.t(r) - d{1}.t(r(1)), ed)

lgd = {'centralized', 'distributed'};
legend(lgd)


%%
r = 200:600;

lc = length(c);
ld = length(d);

errorvc = 0;
for i = 1:lc
    errorvc = errorvc + sqrt(mean((c{i}.v(r) - c{i}.vref(r)).^2));
end
errorvc = errorvc/lc;

errorvd = 0;
for i = 1:ld
    errorvd = errorvd + sqrt(mean((d{i}.v(r) - d{i}.vref(r)).^2));
end
errorvd = errorvd/ld;

errortc = 0;
for i = 2:lc
    errortc = errortc + sqrt(mean((c{i}.timegap(r) - 1).^2));
end
errortc = errortc/(lc - 1);

errortc2 = 0;
for i = 2:lc
    temptg = c{i}.timegap(r) - 1;
    errortc2 = errortc2 + sqrt(mean((temptg(abs(temptg) < 10)).^2));
end
errortc2 = errortc2/(lc - 1);

errortd = 0;
for i = 2:ld
    errortd = errortd + sqrt(mean((d{i}.timegap(r) - 1).^2));
end
errortd = errortd/(ld - 1);

accc = 0;
for i = 1:lc
    accc = accc + sqrt(mean(c{i}.acc(r).^2));
end
accc = accc/lc;

accd = 0;
for i = 1:ld
    accd = accd + sqrt(mean(d{i}.acc(r).^2));
end
accd = accd/ld;

