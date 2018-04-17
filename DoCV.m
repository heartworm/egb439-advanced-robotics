function [obstacle_map, goal_location] = DoCV(image)
%UNTITLED Summary of this function goes here
%   return obstacle map as binary image
%   return goal_coord in row, col format

    im_sum = sum(image, 3);
    im_c = zeros(size(image));
    % Colour intensity im_c_[r|g|b]
    im_c(:,:,1) = image(:,:,1) ./ im_sum;
    im_c(:,:,2) = image(:,:,2) ./ im_sum;
    im_c(:,:,3) = image(:,:,3) ./ im_sum;
    idisp(im_c);
    % im_b = binary image
    im_b_grass = im_c(:,:,2) > 0.4;
    im_b_grass = ierode(im_b_grass, ones(5));
    
    im_b_goal = im_c(:,:,2) > 0.35 & im_c(:,:,1) > 0.35;
    
    figure();
    idisp(im_b_goal);

    im_b_grass = im_b_grass | idilate(im_b_goal, ones(15));
    
    im_b_goal_eroded = ierode(im_b_goal, ones(3));
    
    blobs_goal = iblobs(im_b_goal_eroded, 'class', 1);
    [~, ind_goal_largest] = max(blobs_goal.area);
    blob_goal = blobs_goal(ind_goal_largest);
    
%     moments_goal = imoments(im_b_goal_eroded);

    goal_location = round([blob_goal.vc, blob_goal.uc]);
    
    obstacle_map = ~im_b_grass;

end

