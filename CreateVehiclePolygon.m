function V = CreateVehiclePolygon(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
V = [AX, AY; BX, BY; CX, CY; DX, DY; AX, AY];
end