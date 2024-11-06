
v_body = [0; 3; 3; 0.05; 0; 0];  
initial_orientation = [0; deg2rad(-10); 0];  % (roll, pitch, yaw)




initial_state = [0; 0; 0; initial_orientation];  % [position; orientation]
tspan = [0 150];  

[t, state_trajectory] = ode45(@(t, state) vehicle_kinematics(t, state, v_body), tspan, initial_state);

position_trajectory = state_trajectory(:, 1:3);

figure;
plot3(position_trajectory(:,1), position_trajectory(:,2), position_trajectory(:,3));
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory of the Underwater Vehicle');
grid on;

% Final position and orientation
final_position = position_trajectory(end, :)
final_orientation = rad2deg(state_trajectory(end, 4:6))


function R = rotation_matrix(roll, pitch, yaw)
    R_roll = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    R_pitch = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R_yaw = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = R_yaw * R_pitch * R_roll;
end

% Transform velocity to inertial frame
function v_inertial = transform_velocity(v_body, orientation)
    R = rotation_matrix(orientation(1), orientation(2), orientation(3));
    v_inertial_linear = R * v_body(1:3); 
    v_inertial = [v_inertial_linear; v_body(4:6)]; 
end


function dstate = vehicle_kinematics(~, state, v_body)
    % position = state(1:3);
    orientation = state(4:6);
   
    v_inertial = transform_velocity(v_body, orientation);
    
    position_dot = v_inertial(1:3);  
    orientation_dot = v_body(4:6); 
    dstate = [position_dot; orientation_dot];
end

