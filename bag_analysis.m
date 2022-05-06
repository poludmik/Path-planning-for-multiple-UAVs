bag = rosbag("C:\Users\micha\homeworks\BC\camp_tests\mikhail_bag_files\23_2022_04_28_17_55_30_louka_3_s_rviz\_2022-04-28-17-58-01.bag");
bagselect1 = select(bag,"Topic","/visualization_marker_array");

msgs = readMessages(bagselect1, "DataFormat", "struct");
size(msgs)

coords = [];
starts = [];

for i = 1:size(msgs)
    
    array = msgs{i}.Markers;

    for j = 1:size(array, 2)
    
        if array(j).Type == 2
            if array(j).Pose.Position.X ~= 0
            coords = [coords [array(j).Pose.Position.X 
                array(j).Pose.Position.Y]];
            else
            starts = [starts [array(j).Pose.Position.X 
                array(j).Pose.Position.Y]];
            end
        end

    end

end

f = figure;
scatter(coords(2, :), coords(1, :), 'LineWidth', 1.2);
hold on;
scatter(starts(2, :), starts(1, :), 'LineWidth', 2);
xlabel("Distance to goal in Y axis [m]");
ylabel("Distance to goal in X axis [m]");
xlim([-15.3 0.2]);
ylim([-2 0.8]);
legend('Position deviations', 'Start and goal references');


grid on;
grid minor;
f.Position = [700 300 900 400];






