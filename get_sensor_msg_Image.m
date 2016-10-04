function q = get_sensor_msg_Image(msg)

if strcmp(msg.encoding,'bgra8')
    q = reshape(msg.data,[4,msg.width,msg.height]);
    q = permute(q,[3,2,1]);
    q = q(:,:,[3,2,1,4]);
end
