hold off;
close all;

colors = [
    0.0000    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;
    0.6350    0.0780    0.1840;
    0.6480    0.8150    0.2390;
    0.9290    0.3880    0.1050;
    0.6000    0.6000    0.8000;
];

width = 0.1;
observer_errors = zeros(num,1);
len = numel(broadcast_history{1}(:,1))-2;

safety_radius = epsilon;
num_steps = numel(Time); 
time_steps = 1:num_steps;
observer_errors_all = zeros(num_steps, N);

for i0 = 1:N 
    for i1 = 1:num_steps
        observer_errors_all(i1, i0) = pdist2(broadcast_history{i0}(i1, 1:2), store_states{i0}(i1, 1:2),'euclidean');
    end
end

max_errors = max(observer_errors_all, [], 2);
mean_errors = mean(observer_errors_all, 2);
std_errors = std(observer_errors_all, 0, 2);

figure('Position', [100 100 800 240]);
hold on;

h_mean = plot(Time(1:num_steps), mean_errors, '-', 'Color', [0.31, 0.71, 0.02], 'LineWidth', 1.5);

h_max = plot(Time(1:num_steps), max_errors, '-', 'Color', [1.00, 0.41, 0.16], 'LineWidth', 2);

upper = mean_errors + std_errors;
lower = mean_errors - std_errors;
h_std =fill([Time, fliplr(Time)], ...
     [upper; flipud(lower)], ...
     [0.7 0.7 0.9], 'FaceAlpha',0.5, 'EdgeColor','none');

h_safety = plot([0 480], [safety_radius safety_radius], 'k:', 'LineWidth',1.5);

legend([h_mean, h_max, h_std, h_safety], ...
       {'Mean Error', 'Max Error', 'Standard Deviation Range', 'Maxmum allowable Observer Error'}, ...
       'Location', 'northwest');

xlabel('${t (s)}$', 'Interpreter', 'latex');
ylabel(sprintf('Observation Error $(m)$', i0, i0), 'Interpreter', 'latex');

grid on;
set(gca, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 12);
xlim([Time(1), Time(num_steps)]);
