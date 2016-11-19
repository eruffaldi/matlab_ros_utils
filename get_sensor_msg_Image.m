function q = get_sensor_msg_Image(msg)

if strcmp(msg.encoding,'bgra8')
    q = reshape(msg.data,[4,msg.width,msg.height]);
    q = permute(q,[3,2,1]);
    q = q(:,:,[3,2,1,4]);
elseif strcmp(msg.encoding,'bgr8')
    q = reshape(msg.data,[3,msg.width,msg.height]);
    q = permute(q,[3,2,1]);
    q = q(:,:,[3,2,1]);
elseif strcmp(msg.encoding,'16UC1')
    q = typecast(msg.data,'uint16');
    q = reshape(q,[msg.width,msg.height])';
end
