% Previously extract on a single file all fitnesses to analyze removing
% all the comments.

% CONSTANTS
n_cols = 4;
formatSpec = '%u %f %f %f';
file = fopen('fitnessList', 'r');
start = 2;

% VARIABLES
n_fitness = 3;
n_generations = 100;

% PROGRAM
x = 1:1:(n_generations);
size = [n_cols Inf];
A = fscanf(file, formatSpec, size);
A = A';
from = 1;
j = 1;
for i = 1:n_fitness
    figure(j);
    f_max = A(from:(from + n_generations - 1), 2);
    f_avg = A(from:(from + n_generations - 1), 3);
    f_min = A(from:(from + n_generations - 1), 4);
    f = [f_max, f_avg, f_min];
%     h = stem(x, f, 'filled', 'LineStyle', 'none', 'Marker', '.');
    h = plot(x, f);
    xlabel('Generations');
    title ('Fitness');
    [h(1).Color] = deal('red');
    [h(2).Color] = deal('green');
    [h(3).Color] = deal('blue');
    legend('F_m_a_x','F_a_v_g', 'F_m_i_n')
    from = from + n_generations;
    j = j + 1;
end