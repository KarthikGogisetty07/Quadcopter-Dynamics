function plot_data(data)
    figure; plots = [subplot(2, 1, 1), subplot(2, 1, 2)];
    subplot(plots(1));
    multiplots(data, data.x);
    xlabel('Time (s)');
    ylabel('Linear Displacement (m)');
    title('Linear Displacement');
    
    subplot(plots(2));
    multiplots(data, rad2deg(data.theta));
    xlabel('Time (s)');
    ylabel('Angular Displacement (deg)');
    title('Angular Displacement');
end    
function multiplots(data, values)
    % Select the parts of the data to plot.
    times = data.t(:, 1:end);

    values = values(:, 1:end);

    % Ploting using R = x-coordinate , B = y-coordinate, G = z-coordinate 
    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b.');
    
    % Setting axis limits.
    xmin = min(data.t);
    xmax = max(data.t);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values)) + 1;
    axis([xmin xmax ymin ymax]);
end