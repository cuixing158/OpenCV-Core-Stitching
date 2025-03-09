function uv = FisheyeProjectorForwardFcn(xy,K,R,scale)
% Brief:
%    This function calculates the forward projection of world coordinates (xy)
%    to fisheye camera image coordinates (uv).
%
% Details:
%    The function takes the world coordinates (xy), the camera intrinsic matrix (K),
%    and the rotation matrix (R), and computes the corresponding image coordinates (uv).
%    The scale parameter is used to normalize the image coordinates after the projection.
%
% Syntax:
%     uv = FisheyeProjectorForwardFcn(xy,K,R,scale)
%
% Inputs:
%    xy - [m,2] size,[double] type, 输入点集坐标
%    K - [3,3] size,[double] type, 相机内参矩阵
%    R - [3,3] size,[double] type, 旋转矩阵R
%    scale - [1,1] size,[double] type, 用于归一化图像坐标的缩放因子
%
% Outputs:
%    uv - [m,2] size,[double] type, 输出点集坐标
%
% Example:
%    None
%
% See also:
%
% Reference:
%     https://github.com/opencv/opencv/blob/57a78cb9dfd591c95efe4afd2953db97993d3702/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L318
%

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         03-Mar-2025 16:54:25
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
      xy (:,2) double
      K (3,3) double
      R (3,3) double
      scale (1,1) double = mean([K(1,1),K(2,2)]);
end

num = size(xy,1);

xyz_ = R/K*[xy';ones(1,num)];
xyz_ = xyz_';

x_ = xyz_(:,1);
y_ = xyz_(:,2);
z_ = xyz_(:,3);

u_ = atan2(x_,z_);
v_ = pi - acos(y_./sqrt(x_.^2+y_.^2+z_.^2));

uv = scale.*v_.*[cos(u_),sin(u_)];

end