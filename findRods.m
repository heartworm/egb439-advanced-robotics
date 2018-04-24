function [rods] = findRods(image)   
    image_size = size(image);
    im_sum = sum(image, 3);
    im_g = max(image, [], 3);
    im_c = zeros(size(image));
    % Colour intensity im_c_[r|g|b]
    im_c(:,:,1) = image(:,:,1) ./ im_sum;
    im_c(:,:,2) = image(:,:,2) ./ im_sum;
    im_c(:,:,3) = image(:,:,3) ./ im_sum;
    idisp(im_c);
    
    im_thresh_yellow = createMaskYellow(im_c);
    im_thresh_red = createMaskRed(im_c);
    im_thresh_blue = createMaskBlue(im_c);
    
    YELLOW = 3;
    BLUE = 2;
    RED = 1;
    
    im_thresh = im_thresh_yellow * YELLOW + im_thresh_blue * BLUE + im_thresh_red * RED;
    blobs = iblobs(im_thresh, 'area', [100, inf]);
    blobs = blobs(blobs.class ~= 0);
    
    blobs_yellow = blobs(blobs.class == YELLOW);
    blobs_red = blobs(blobs.class == RED);
    blobs_blue = blobs(blobs.class == BLUE);
    
    num_rods = min([length(blobs_yellow), length(blobs_red), length(blobs_blue)]);
    
    
    template_rod = struct('blob_yellow', {}, 'blob_blue', {}, 'blob_green', {}, 'uc', {}, 'vc', {}, 'box', {});
    
    rods = [];
    for i = 1:num_rods
        rod.blob_yellow = blobs_yellow(i);
        
        other_blobs = [blobs_red, blobs_blue];
        other_blobs_distances = abs(other_blobs.uc - rod.blob_yellow.uc);
        [~, other_blobs_sorted_ind] = sort(other_blobs_distances);
        other_blobs_sorted = other_blobs(other_blobs_sorted_ind);
        
        red_blobs_sorted = other_blobs_sorted(other_blobs_sorted.class == RED);
        blue_blobs_sorted = other_blobs_sorted(other_blobs_sorted.class == BLUE);
        
        rod.blob_red = red_blobs_sorted(1);
        rod.blob_blue = blue_blobs_sorted(1);
        
        rod.blobs = [rod.blob_red, rod.blob_blue, rod.blob_yellow];
        
        [~, sorted_vc_ind] = sort(rod.blobs.vc);
        sorted_vc_ind = fliplr(sorted_vc_ind);
        
        rod.code = bin2dec(reshape(dec2bin(sorted_vc_ind)', 1, []));
        
        tl = [min(rod.blobs.uc), min(rod.blobs.vc)];
        br = [max(rod.blobs.uc), max(rod.blobs.vc)];
        
        rod.uc = mean(rod.blobs.uc);
        rod.vc = mean(rod.blobs.vc);
        rod.box = [tl; br];
        
        rod_dims = rod.box(2,:) - rod.box(1,:);
        ROD_HEIGHT_IRL = 0.15;
        CAMERA_FOV = deg2rad([62.2, 48.8]);
        
        rod_height_proportion = rod_dims(2) / size(image, 1);
        rod_height_angle = rod_height_proportion * CAMERA_FOV(2);

        rod.range = ROD_HEIGHT_IRL / tan(rod_height_angle);
        
        
        
        rod.bearing = ((image_size(2) / 2) - rod.uc) ...
                          / image_size(2) ...
                          * CAMERA_FOV(1);
        
        rods = [rods, rod];
    end
    
end
