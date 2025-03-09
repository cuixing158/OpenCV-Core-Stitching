function xy = SphericalProjectorInverseFcn(uv,K,R,scale)
% Brief:
%    This function calculates the inverse projection of spherical camera coordinates
%    from the image plane (uv) to the world plane (xy).
%
% Details:
%    The function takes the image coordinates (uv), the camera intrinsic matrix (K),
%    and the rotation matrix (R), and computes the corresponding world coordinates (xy).
%    The scale parameter is used to normalize the image coordinates before the projection.
%
% Syntax:
%     xy = SphericalProjectorInverseFcn(uv,K,R,scale)
%
% Inputs:
%    uv - [m,2] size,[double] type, Image coordinates in the spherical camera image plane.
%    K - [3,3] size,[double] type, 相机内参矩阵 (Camera intrinsic matrix).
%    R - [3,3] size,[double] type, 旋转矩阵R (Rotation matrix).
%    scale - [1,1] size,[double] type, 用于归一化图像坐标的缩放因子 (Scale factor for normalizing the image coordinates).
%
% Outputs:
%    xy - [m,2] size,[double] type, 世界坐标 (World coordinates) corresponding to the input image coordinates.
%
% Example:
%    None
% 
% See also: geometricTransform2d
%
% References:
% https://github.com/opencv/opencv/blob/5c6c6af4ec1f35e1c20f366258793f711e428c1a/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L266

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         03-Mar-2025 17:03:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
    uv (:,2) double
    K (3,3) double
    R (3,3) double
    scale (1,1) double=mean([K(1,1),K(2,2)]);
end

% scale = mean([K(1,1),K(2,2)]);
uv = uv./scale;

sinv = sin(pi-uv(:,2));

x_ = sinv.*sin(uv(:,1));
y_ = cos(pi-uv(:,2));
z_ = sinv.*cos(uv(:,1));

xyz = K*R'*[x_';y_';z_'];
xyz = xyz';

mask = xyz(:,3)>0;
xyz(mask,:) = xyz(mask,:)./xyz(mask,3);
xyz(~mask,1:2) = -1;

xy = xyz(:,1:2);
end