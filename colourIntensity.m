function im_c = colourIntensity(image)
    im_sum = sum(image, 3);
    im_c = zeros(size(image));
    % Colour intensity im_c_[r|g|b]
    im_c(:,:,1) = image(:,:,1) ./ im_sum;
    im_c(:,:,2) = image(:,:,2) ./ im_sum;
    im_c(:,:,3) = image(:,:,3) ./ im_sum;
end

