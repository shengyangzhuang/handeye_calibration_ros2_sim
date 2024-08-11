clear
clc
close all
%%
addpath(genpath('/home/szhuang/MATLAB Add-Ons/yamlmatlab'));
yaml_file = 'handeye_result_simulation_profile.yaml';
data = yaml.ReadYaml(yaml_file);

for i=1:13
    rotation{i,1} = data.transforms{1,i}.rotation;
    translation{i,1} = data.transforms{1,i}.translation;
end
flattenedRotation = cellfun(@cell2mat, rotation, 'UniformOutput', false);
flattenedTranslation = cellfun(@cell2mat, translation, 'UniformOutput', false);
rotation = cell2mat(flattenedRotation);
translation = cell2mat(flattenedTranslation);

r_x_real = -0.0002815440309169026;
r_y_real = 0.00028154403091679164;
r_z_real = -0.7071067251362829;
r_w_real = 0.7071067251362828;

t_x_real = -0.05;
t_y_real = 0;
t_z_real = 0;

%%
x = 3:15;
for i=1:13
    y(1,i) = abs(rotation(i,1) - r_x_real);
    y(2,i) = abs(rotation(i,2) - r_y_real);
    y(3,i) = abs(rotation(i,3) - r_z_real);
    y(4,i) = abs(rotation(i,4) - r_w_real);
    e(1,i) = abs(translation(i,1) - t_x_real);
    e(2,i) = abs(translation(i,2) - t_y_real);
    e(3,i) = abs(translation(i,3) - t_z_real);
end
% plot(x,y(1,:),'-o', 'LineWidth', 2);
% hold on;
% plot(x,y(2,:),'-o', 'LineWidth', 2);
% hold on;
% plot(x,y(3,:),'-o', 'LineWidth', 2);
% hold on;
% plot(x,y(4,:),'-o', 'LineWidth', 2);
% Plot cumulative errors
figure;
plot(x, y(1,:) + y(2,:) + y(3,:) + y(4,:), '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0, 0.4470, 0.7410]);
hold on;
plot(x, e(1,:) + e(2,:) + e(3,:), '-*', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.8500, 0.3250, 0.0980]);

% Enhance plot appearance
grid on;
legend({'Cumulated rotation quaternion errors $\Sigma_{i=x,y,z,w} |\Delta r_i|$', 'Cumulated translation vector errors $\Sigma_{i=x,y,z} |\Delta t_i|$'}, 'Interpreter', 'latex', 'FontSize', 12);
title('Cumulated Hand-Eye Calibration Errors over Samples', 'FontSize', 14);
xlabel('Sample Number', 'FontSize', 12);
ylabel('Cumulated Errors', 'FontSize', 12);
xlim([3 13]);
%ylim([0 max(max(y(1,:) + y(2,:) + y(3,:) + y(4,:)), max(e(1,:) + e(2,:) + e(3,:)))*1.1]);

% Display the plot
hold off;
%%
% figure;
% 
% x = 1:7;
% for i = 1:7
%     y(1, i) = abs(rotation(i, 1) - r_x_real);
%     y(2, i) = abs(rotation(i, 2) - r_y_real);
%     y(3, i) = abs(rotation(i, 3) - r_z_real);
%     y(4, i) = abs(rotation(i, 4) - r_w_real);
% end
% 
% % Create subplots for each comparison
% subplot(2, 2, 1);
% plot(x, y(1,:), '-o', 'LineWidth', 2);
% title('Rotation vs r_x_real');
% xlabel('x');
% ylabel('|rotation - r_x_real|');
% grid on;
% 
% subplot(2, 2, 2);
% plot(x, y(2,:), '-o', 'LineWidth', 2);
% title('Rotation vs r_y_real');
% xlabel('x');
% ylabel('|rotation - r_y_real|');
% grid on;
% 
% subplot(2, 2, 3);
% plot(x, y(3,:), '-o', 'LineWidth', 2);
% title('Rotation vs r_z_real');
% xlabel('x');
% ylabel('|rotation - r_z_real|');
% grid on;
% 
% subplot(2, 2, 4);
% plot(x, y(4,:), '-o', 'LineWidth', 2);
% title('Rotation vs r_w_real');
% xlabel('x');
% ylabel('|rotation - r_w_real|');
% grid on;
