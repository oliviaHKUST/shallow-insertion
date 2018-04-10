% Given 
% 1. card_length_all(length of the card)
% 2. card_length_outside (the length of the card outside the gripper)
% 3. gripper_length (length of the gripper)
% 4. gripper_width (open width of the gripper)
% 5. card_theta (angle between the card and the ground)

% Calculate
% 1. the gripper point (position of the end effector)
% 2. gripper theta (angle between the end effector's z axis and the ground)

% Restriction
% 1. gripper_width < card_length_inside (the length of the card inside the gripper)
% 2. gripper does not hit the ground

card_length_all = 60.0;
gripper_length = 50;


for i = (6 : 28)
    
    card_theta = pi / i;
    if (i == 28)
        card_theta = 0;
    end
    card_length_outside = 30.0;
    if (i > 3)
        card_length_outside = card_length_outside + (i-3)/5;
    end
    gripper_width = 25.0;

    card_length_inside = card_length_all - card_length_outside;
    if (gripper_width > card_length_inside)
        fprintf('error:gripper width:%dmm is greater than card length inside gripper:%dmm\nplease change parameters\n================\n' ...
            , gripper_width, card_length_inside);
        return
    end
    gripper_width;
    card_length_inside;
    gripper_alpha = acos(gripper_width/card_length_inside);
    gripper_alpha / pi * 180;
    gripper_alpha_complementary = pi/2 - gripper_alpha;
    gripper_alpha_complementary / pi * 180;
    gripper_theta = gripper_alpha_complementary + card_theta;
    gripper_theta / pi * 180;

    gripper_side = sqrt(card_length_inside^2 - gripper_width^2);
    gripper_length_left = gripper_length - gripper_side;

    gripper_left_side_1_x = cos(card_theta) * card_length_outside;
    gripper_left_side_1_y = sin(card_theta) * card_length_outside;

    gripper_left_side_2_x = gripper_left_side_1_x + cos(gripper_theta) * gripper_side;
    gripper_left_side_2_y = gripper_left_side_1_y + sin(gripper_theta) * gripper_side;

    gripper_left_side_3_x = gripper_left_side_2_x + cos(gripper_theta) * gripper_length_left;
    gripper_left_side_3_y = gripper_left_side_2_y + sin(gripper_theta) * gripper_length_left;


    card_2_x = cos(card_theta) * card_length_all;
    card_2_y = sin(card_theta) * card_length_all;

    card_x = [0, card_2_x];
    card_y = [0, card_2_y];

    gripper_left_x = [gripper_left_side_1_x, gripper_left_side_2_x, gripper_left_side_3_x];
    gripper_left_y = [gripper_left_side_1_y, gripper_left_side_2_y, gripper_left_side_3_y];

    gripper_right_side_2_x = card_2_x - cos(gripper_theta) * gripper_side;
    gripper_right_side_2_y = card_2_y - sin(gripper_theta) * gripper_side;
    
    if (gripper_right_side_2_y < 0)
        fprintf('error: gripper hits the ground\n');
        return
    end

    gripper_right_side_3_x = card_2_x + cos(gripper_theta) * gripper_length_left;
    gripper_right_side_3_y = card_2_y + sin(gripper_theta) * gripper_length_left;


    gripper_right_x = [card_2_x, gripper_right_side_2_x, gripper_right_side_3_x];
    gripper_right_y = [card_2_y, gripper_right_side_2_y, gripper_right_side_3_y];

    gripper_top_x = [gripper_left_side_2_x, cos(card_theta) * card_length_all];
    gripper_top_y = [gripper_left_side_2_y, sin(card_theta) * card_length_all];

    plot(card_x, card_y, 'k-', gripper_left_x, gripper_left_y, 'b-', gripper_top_x, gripper_top_y, 'b--', gripper_right_x, gripper_right_y, 'b-');

    end_effector_x = (gripper_left_side_3_x + gripper_right_side_3_x) / 2;
    end_effector_y = (gripper_left_side_3_y + gripper_right_side_3_y) / 2;

    axis equal
    axis([0, 200, 0, 200]) 

    fprintf('position of the end effector: [%f, %f]\n', end_effector_x, end_effector_y);
    fprintf('gripper theta: %f\n', gripper_theta / pi * 180);
    
    pause(1)
end
