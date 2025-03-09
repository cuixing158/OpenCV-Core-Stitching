function xy = CylindricalProjectorInverseFcn(uv,K,R,scale)
% Brief:
%    将圆柱投影坐标逆转换为二维图像坐标。
%
% Details:
%    该函数首先对输入的圆柱投影坐标 uv 根据比例因子 scale 进行归一化，
%    利用三角函数将柱面坐标（角度和垂直位移）转换为三维空间中的方向向量。
%    接着，通过相机内参矩阵 K 与旋转矩阵 R 的联合反变换，
%    将方向向量映射到图像平面上得到预估的二维坐标。
%    对于转换得到的三维点，当其 z 坐标大于零时，将各分量归一化以确保结果有效，
%    否则将结果置为无效标记（-1）。
%
% Syntax:
%     xy = CylindricalProjectorInverseFcn(uv,K,R,scale)
%
% Inputs:
%    uv - [m,2] size,[double] type,Description
%    K - [3,3] size,[double] type,相机内参矩阵
%    R - [3,3] size,[double] type,旋转矩阵R
%
% Outputs:
%    xy - [m,2] size,[double] type, 输出的二维图像坐标。
%
% Example:
%    None
%
% See also: None
%
% References:
%    https://github.com/opencv/opencv/blob/57a78cb9dfd591c95efe4afd2953db97993d3702/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L299
%
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
    scale (1,1) double = mean([K(1,1),K(2,2)])
end

uv = uv./scale;

x_ = sin(uv(:,1));
y_ = uv(:,2);
z_ = cos(uv(:,1));

xyz = K/R*[x_,y_,z_]';
xyz = xyz';

mask = xyz(:,3) > 0;
xyz(mask,:) = xyz(mask,:) ./ xyz(mask,3);
xyz(~mask,1:2) = -1;

xy = xyz(:,1:2);
end


