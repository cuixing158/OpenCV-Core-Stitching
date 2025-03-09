function xy =  StereoProjectorInverseFcn(uv,K,R,scale)
% Brief:
%    将立体投影坐标逆转换为二维图像坐标。
%
% Details:
%    该函数首先对输入的立体投影坐标 uv 根据比例因子 scale 进行归一化，
%    利用三角函数将立体投影坐标转换为三维空间中的方向向量。其中，
%    u_ 通过 atan2 函数计算得到平面角，r 代表 uv 的欧式模长，
%    v_ 则通过 2*atan(1./r) 得到倾斜角。接着，根据方向向量的
%    正弦和余弦分量计算出三维点的 x、y、z 分量，并通过相机内参矩阵 K 和
%    旋转矩阵 R 的联合反变换，将方向向量映射到图像平面上。对生成的三维点，
%    当 z 分量大于零时进行归一化处理，保证映射结果有效；否则，输出无效标记（-1）。
%
% Syntax:
%     xy = StereoProjectorInverseFcn(uv,K,R,scale)
%
% Inputs:
%    uv    - [m,2] size,[double] type, 立体投影坐标，通常包含了平面的两维信息。
%    K     - [3,3] size,[double] type, 相机内参矩阵。
%    R     - [3,3] size,[double] type, 旋转矩阵 R，默认值为单位矩阵。
%    scale - [1,1] double, 投影比例因子, 默认值为 mean([K(1,1),K(2,2)])。
%
% Outputs:
%    xy    - [m,2] size,[double] type, 输出的二维图像坐标。
%
% Example:
%    None
%
% See also: None
%
% References:
%     https://github.com/opencv/opencv/blob/57a78cb9dfd591c95efe4afd2953db97993d3702/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L371

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
r = vecnorm(uv,2,2);
v_ = 2*atan(1./r);

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


