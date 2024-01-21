%% Formation benchmark
clc
close all
clear variables

disp('********************************************')
disp('*               Robotic Park               *')
disp('* A benchmark on formation control of MARS *')
disp('********************************************')

%% Problem 2
dataset_folder = '2024-01-20-22-09';
cd(dataset_folder)

% Read topology
fid = fopen('default_config.yaml', 'r' );
tline = fgetl(fid);
config_file = [];
i=1;
flag = 0;
while ischar(tline)
    if contains(tline, 'name: ')
        string_aux = split(tline);
        robot_name = string_aux(3);
        aux = robot_name;
        while strcmpi(robot_name, aux) && ischar(tline)
            tline = fgetl(fid);
            if ischar(tline)
                if contains(tline, 'relationship: ')
                    rel = split(strcat(tline,','));
                    dataset.(robot_name{i}).relationship = split(rel(3:end),["_",","]);
                    break
                end
                if contains(tline, 'name: ')
                    break
                end
            end
        end
    else
        tline = fgetl(fid);
    end
end
fclose(fid);

% Read data
robots = fieldnames(dataset);
t = [];
tf = [];
disp('Reading data')
for i=1:length(robots)
    disp(robots{i})
    figure('Name',robots{i})
    hold on
    dataset.(robots{i}).local_pose = readtable(strcat(robots{i},'-local_pose.csv'));
    for j=1:length(dataset.(robots{i}).relationship(:,1))
        dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).data = readtable(char(strcat(robots{i},'-',dataset.(robots{i}).relationship(j,1),'-data.csv')));
        dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).error = readtable(char(strcat(robots{i},'-',dataset.(robots{i}).relationship(j,1),'-error.csv')));
        dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae = readtable(char(strcat(robots{i},'-',dataset.(robots{i}).relationship(j,1),'-iae.csv')));
        plot(dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).data.Timestamp, abs(dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).data.Data - str2double(char(dataset.(robots{i}).relationship(j,2)))))
    end
    dataset.(robots{i}).global_error = readtable(char(strcat(robots{i},'-global_error.csv')));
    t = [t dataset.(robots{i}).local_pose.Timestamp(1) ];
    tf = [tf dataset.(robots{i}).global_error.Timestamp(1) ];
    plot(dataset.(robots{i}).global_error.Timestamp, dataset.(robots{i}).global_error.Data)
    grid minor
    xlim([dataset.(robots{i}).global_error.Timestamp(1) dataset.(robots{i}).global_error.Timestamp(end)])
    xlabel('Time (s)')
    ylabel('Error (m)')
end

cd ..

t_init = min(t);
t_start = min(tf);
disp('Get index')
for i=1:length(robots)
    dataset.(robots{i}).local_pose.Timestamp = (dataset.(robots{i}).local_pose.Timestamp - t_init)*10^-9;
    dataset.(robots{i}).global_error.Timestamp = (dataset.(robots{i}).global_error.Timestamp - t_init)*10^-9;
    for j=1:length(dataset.(robots{i}).relationship(:,1))
        dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).data.Timestamp = (dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).data.Timestamp - t_init)*10^-9;
        dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).error.Timestamp = (dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).error.Timestamp - t_init)*10^-9;
        dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Timestamp = (dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Timestamp - t_init)*10^-9;
    end
end

dataset.J2 = 0.0;
for i=1:length(robots)
    % Settling time
    delta_signal = abs(dataset.(robots{i}).global_error.Data(1)-dataset.(robots{i}).global_error.Data(end))*0.02;
    for k = 1:length(dataset.(robots{i}).global_error.Data)
        if dataset.(robots{i}).global_error.Data(k) > dataset.(robots{i}).global_error.Data(end)+delta_signal || dataset.(robots{i}).global_error.Data(k) < dataset.(robots{i}).global_error.Data(end)-delta_signal
            dataset.(robots{i}).t_s = dataset.(robots{i}).global_error.Timestamp(k);
            last = k;
        end
    end
    % local_pose
    % n_{m,t}: Number of published messages in the network
    [ind1, ~] = find(dataset.(robots{i}).local_pose.Timestamp >= dataset.(robots{i}).global_error.Timestamp(1));
    ind = ind1(dataset.(robots{i}).local_pose.Timestamp(ind1:end)<dataset.(robots{i}).t_s);
    dataset.(robots{i}).n_m = length(ind);
    % global error
    % n_{u,t}: Number of controller executions
    dataset.(robots{i}).n_u = length(dataset.(robots{i}).global_error.Timestamp(1:last));
    % IAE_{i}
    dataset.(robots{i}).iae = 0.0;
    for j=1:length(dataset.(robots{i}).relationship(:,1))
        dataset.(robots{i}).iae = dataset.(robots{i}).iae + 0.5 * dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Data(last);
    end
    % ITAE_{i}
    for j=1:length(dataset.(robots{i}).relationship(:,1))
        [ind1, ~] = find(dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Timestamp >= dataset.(robots{i}).global_error.Timestamp(1));
        ind = ind1(dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Timestamp(ind1:end)<dataset.(robots{i}).t_s);
        dataset.(robots{i}).itae = 0.0;
        for k=1:length(ind)
            dataset.(robots{i}).itae = dataset.(robots{i}).itae + 0.5 * (dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Data(k+1)-dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Data(k))*(dataset.(robots{i}).(char(dataset.(robots{i}).relationship(j,1))).iae.Timestamp(k)-dataset.(robots{i}).global_error.Timestamp(1));
        end
    end
    dataset.(robots{i}).t_s = dataset.(robots{i}).t_s - dataset.(robots{i}).global_error.Timestamp(1);

    % J2 Eq. (11)
    dataset.(robots{i}).J2 = 0.02 * dataset.(robots{i}).n_m + 0.1 * dataset.(robots{i}).n_u + 0.3 * dataset.(robots{i}).t_s + 0.1 * dataset.(robots{i}).iae + 0.01 * dataset.(robots{i}).itae;
    disp(robots{i})
    results = ['n_m=',num2str(dataset.(robots{i}).n_m),' n_u=',num2str(dataset.(robots{i}).n_u), ' t_s=',num2str(dataset.(robots{i}).t_s), ' IAE=',num2str(dataset.(robots{i}).iae), ' ITAE=',num2str(dataset.(robots{i}).itae)];
    disp(results)
    results = ['J2=',num2str(dataset.(robots{i}).J2)];
    disp(results)
    dataset.J2 = dataset.J2 + dataset.(robots{i}).J2;
end

results = ['Benchmark result (Problem 2): J2=',num2str(dataset.J2)];
disp(results)
