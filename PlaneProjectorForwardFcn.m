function uv = PlaneProjectorForwardFcn(xy,K,R,t,scale)
% Brief:
%    This function calculates the forward projection of world coordinates (xy)
%    to plane camera image coordinates (uv).
%
% Details:
%    The function takes the world coordinates (xy), the camera intrinsic matrix (K),
%    the rotation matrix (R), and the translation vector (t), and computes the corresponding
%    image coordinates (uv). The scale parameter is used to normalize the image coordinates
%    after the projection.
%
% Syntax:
%     uv = PlaneProjectorForwardFcn(xy,K,R,t,scale)
%
% Inputs:
%    xy - [m,2] size,[double] type, 输入点集坐标
%    K - [3,3] size,[double] type, 相机内参矩阵 (Camera intrinsic matrix).
%    R - [3,3] size,[double] type, 旋转矩阵R (Rotation matrix).
%    t - [m,3] size,[double] type, 平移向量t (Translation vector).
%    scale - [1,1] size,[double] type, 用于归一化图像坐标的缩放因子 (Scale factor for normalizing the image coordinates).
%
% Outputs:
%    uv - [m,2] size,[double] type, 输出点集坐标 (Image coordinates) corresponding to the input world coordinates.
%
% Example:
%    None
% 
% See also: PlaneProjectorInverseFcn
%
% References:
%   https://github.com/opencv/opencv/blob/5c6c6af4ec1f35e1c20f366258793f711e428c1a/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L222

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
    R (3,3) double = eye(3)
    t (:,3) double = zeros(size(xy,1),3)
    scale (1,1) double =mean([K(1,1),K(2,2)])
end

nums = size(xy,1);
xyz_ = R/K*[xy';ones(nums,1)'];
xyz_ = xyz_';

x_ = t(:,1)+xyz_(:,1)./xyz_(:,3).*(1-t(:,3));
y_ = t(:,2)+xyz_(:,2)./xyz_(:,3).*(1-t(:,3));

% scale = mean([K(1,1),K(2,2)]);
uv = scale*[x_,y_];
end