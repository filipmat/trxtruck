function cell_array = parse_recording_bag(filename)

bag = rosbag(filename);

topics = bag.AvailableTopics.Properties.RowNames;
n = length(topics);

cell_array = cell(n, 1);

for i = 1:n
    x = zeros(n, 1);
    y = zeros(n, 1);
    
    vehicle_struct = struct;
    
    sub_bag = select(bag, 'Topic', topics{i});
    msgs = readMessages(sub_bag);
    
    ros_time = timeseries(sub_bag).Time;
    own_time = cell2mat(cellfun(@(c) c.Time, msgs, 'UniformOutput', false));
    
    if mean(ros_time(2:end) - ros_time(1:end - 1)) < 0.01
        t = own_time;
    else
        t = ros_time;
    end
    
    vehicle_struct.id = msgs{1}.Id;
    
    vehicle_struct.t = t;
    
    vehicle_struct.x = cell2mat(cellfun(@(c) c.X, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.y = cell2mat(cellfun(@(c) c.Y, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.yaw = cell2mat(cellfun(@(c) c.Yaw, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.v = cell2mat(cellfun(@(c) c.V, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.pos = cell2mat(cellfun(@(c) c.Pos, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.vref = cell2mat(cellfun(@(c) c.Vref, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.timegap = cell2mat(cellfun(@(c) c.Timegap, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.acc = cell2mat(cellfun(@(c) c.Acc, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.path_error = cell2mat(cellfun(@(c) c.PathError, msgs, ...
        'UniformOutput', false));
    
    vehicle_struct.velocity_input = cell2mat(cellfun(@(c) c.VelocityInput, ...
        msgs, 'UniformOutput', false));
    
    vehicle_struct.steering_input = cell2mat(cellfun(@(c) c.SteeringInput, ...
        msgs, 'UniformOutput', false));
    
    vehicle_struct.gear = cell2mat(cellfun(@(c) c.Gear, msgs, ...
        'UniformOutput', false));
    
    cell_array{i} = vehicle_struct;
    
end