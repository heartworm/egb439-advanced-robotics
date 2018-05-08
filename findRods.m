function [rods] = findRods(image, poseTrans)
    image_size = size(image);
    
%     im_c = colourIntensity(image);
    
    im_thresh_yellow = ierode(idilate(createMaskYellowIm(image), ones(9)), ones(9));
    im_thresh_red = ierode(idilate(createMaskRed(image), ones(9)), ones(9));
    im_thresh_blue = ierode(idilate(createMaskBlue(image), ones(9)), ones(9));
    
    YELLOW = 3;
    BLUE = 2;
    RED = 1;
        
    im_thresh = im_thresh_yellow * YELLOW + im_thresh_blue * BLUE + im_thresh_red * RED;
    blobs = iblobs(im_thresh, 'area', [50, inf]);
    blobs = blobs(blobs.class ~= 0 & blobs.aspect > 0.5);
    
    blobs_yellow = blobs(blobs.class == YELLOW);
    blobs_red = blobs(blobs.class == RED);
    blobs_blue = blobs(blobs.class == BLUE);
    
    num_rods = min([length(blobs_yellow), length(blobs_red), length(blobs_blue)]);
    
    template_rod = struct('blob_yellow', {}, ...
                            'blob_blue', {}, ...
                            'blob_red', {}, ...
                            'blobs', {}, ...
                            'code', {}, ...
                            'uc', {}, ...
                            'vc', {}, ...
                            'box', {}, ... 
                            'range', {}, ...
                            'bearing', {}, ...
                            'height', {}, ...
                            'width', {}, ...
                            'x', {}, ...
                            'y', {});
    
    rods = repmat(template_rod, 0, 0);

    for i = 1:length(blobs_yellow)
        for j = 1:length(blobs_red)
            for k = 1:length(blobs_blue)                
                rod.blob_yellow = blobs_yellow(i);
                rod.blob_red = blobs_red(j);
                rod.blob_blue = blobs_blue(k);
                        rod.blobs = [rod.blob_red, rod.blob_blue, rod.blob_yellow];
        
                [~, sorted_vc_ind] = sort(rod.blobs.vc);
                sorted_vc_ind = fliplr(sorted_vc_ind);

                rod.code = bin2dec(reshape(dec2bin(sorted_vc_ind)', 1, []));

                boxes = rod.blobs.box';
                tl = min(boxes);
                br = max(boxes);

                rod.uc = mean(rod.blobs.uc);
                rod.vc = mean(rod.blobs.vc);
                rod.box = [tl; br];
                
                rod.height = br(2) - tl(2);
                rod.width = br(1) - tl(1);

                rod_dims = rod.box(2,:) - rod.box(1,:);
                ROD_HEIGHT_IRL = 0.155;
                CAMERA_FOV = deg2rad([62.2, 48.8]);

                rod_height_proportion = rod_dims(2) / size(image, 1);
                rod_height_angle = rod_height_proportion * CAMERA_FOV(2);

                rod.range = ROD_HEIGHT_IRL / tan(rod_height_angle);

                rod.bearing = ((image_size(2) / 2) - rod.uc) ...
                                  / image_size(2) ...
                                  * CAMERA_FOV(1);

                rodTrans = poseTrans * transl2(rod.range * cos(rod.bearing), ...
                                                rod.range * sin(rod.bearing));
                rod.x = rodTrans(1,3);
                rod.y = rodTrans(2,3);

                max_blob_width = max(rod.blobs.umax - rod.blobs.umin);
                
                if rod.width < max_blob_width * 1.25 && rod.width * rod.height < 8000
                    rods = [rods, rod]; 
                end     
            end
        end
    end
    
%     [~, clustered_rod_inds] = sort([rods.width]);
%     best_rod_inds = clustered_rod_inds(1:num_rods);
%     rods = rods(best_rod_inds);
    
end
