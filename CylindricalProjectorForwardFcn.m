function uv = CylindricalProjectorForwardFcn(xy,K,R,scale)
% Brief:
%    将二维图像坐标转换为圆柱投影坐标。
%
% Details:
%    该函数首先通过相机内参矩阵K的逆及旋转矩阵R对输入点集xy进行校正和旋转，
%    将二维点映射到相应的三维空间。接着，利用圆柱体投影原理，
%    计算点在圆柱面上的坐标：水平坐标u为点的x和z坐标通过atan2函数计算得到，
%    垂直坐标v则由y坐标与点在xz平面的距离的比例得到。参数scale用于调整
%    投影的尺度，从而适配不同的相机参数设置。
%
% Syntax:
%     uv = CylindricalProjectorForwardFcn(xy,K,R,scale)
%
% Inputs:
%    xy - [m,2] size,[double] type,输入点集坐标
%    K  - [3,3] size,[double] type,相机内参矩阵
%    R  - [3,3] size,[double] type,旋转矩阵R
%    scale - [1,1] double,投影比例因子, 默认值为mean([K(1,1),K(2,2)])
%
% Outputs:
%    uv - [m,2] size,[double] type,输出点集坐标
%
% Example:
%    None
%
% See also:
%
% Reference:
%    https://github.com/opencv/opencv/blob/57a78cb9dfd591c95efe4afd2953db97993d3702/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L287
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

u = scale*atan2(x_,z_);
v = scale*y_./sqrt(x_.^2+z_.^2);
uv = [u,v];
end