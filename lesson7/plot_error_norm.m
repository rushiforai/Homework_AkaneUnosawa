function plot_error_norm(stored_error,stored_time, percentage)
    % Get the length of the data
    data_length = length(stored_error);
    % Get the first "percentage" percent of the data (as an integer)
    data_cut_length = round((percentage/100.0)*data_length);

    % Plot the desired data
    plot(stored_time(1:data_cut_length),stored_error(:,1:data_cut_length))
end