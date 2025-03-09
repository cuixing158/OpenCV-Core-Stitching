function xy =  FisheyeProjectorInverseFcn(uv,K,R,scale)
% Brief:
%    This function calculates the inverse projection of fisheye camera coordinates
%    from the image plane (uv) to the world plane (xy).
%
% Details:
%    The function takes the image coordinates (uv), the camera intrinsic matrix (K),
%    and the rotation matrix (R), and computes the corresponding world coordinates (xy).
%    The scale parameter is used to normalize the image coordinates before the projection.
%
% Syntax:
%     xy =  FisheyeProjectorInverseFcn(uv,K,R,scale)
%
% Inputs:
%    uv - [m,2] size,[double] type, Image coordinates in the fisheye camera image plane.
%    K - [3,3] size,[double] type, Camera intrinsic matrix.
%    R - [3,3] size,[double] type, Rotation matrix.
%    scale - [1,1] size,[double] type, Scale factor for normalizing the image coordinates.
%
% Outputs:
%    xy - [m,2] size,[double] type, World coordinates corresponding to the input image coordinates.
%
% Example:
%    None
%
% See also: None
%
% References:
%     https://github.com/opencv/opencv/blob/57a78cb9dfd591c95efe4afd2953db97993d3702/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L332

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         03-Mar-2025 17:04:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
    uv (:,2) double
    K (3,3) double
    R (3,3) double = eye(3)
    scale (1,1) double =mean([K(1,1),K(2,2)])
end

uv = uv./scale;

u_ = atan2(uv(:,2),uv(:,1));
v_ = vecnorm(uv,2,2);

sinv = sin(pi-v_);
x_ = sinv.*sin(u_);
y_ = cos(pi-v_);
z_ = sinv.*cos(u_);

xyz = K/R*[x_';y_';z_'];
xyz = xyz';
mask = xyz(:,3)>0;

xyz(mask,:) = xyz(mask,:)./xyz(mask,3);
xyz(~mask,1:2) = -1;

xy = xyz(:,1:2);
end


