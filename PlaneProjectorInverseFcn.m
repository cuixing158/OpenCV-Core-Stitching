function xy =  PlaneProjectorInverseFcn(uv,K,R,t,scale)
% Brief:
%    This function calculates the inverse projection of plane coordinates
%    from the image plane (uv) to the world plane (xy).
%
% Details:
%    The function takes the image coordinates (uv), the camera intrinsic matrix (K),
%    the rotation matrix (R), and the translation vector (t), and computes the corresponding
%    world coordinates (xy). The scale parameter is used to normalize the image coordinates
%    before the projection.
%
% Syntax:
%     xy = PlaneProjectorInverseFcn(uv,K,R,t,scale)
%
% Inputs:
%    uv - [m,2] size,[double] type, Image coordinates in the plane camera image plane.
%    K - [3,3] size,[double] type, 相机内参矩阵 (Camera intrinsic matrix).
%    R - [3,3] size,[double] type, 旋转矩阵R (Rotation matrix).
%    t - [m,3] size,[double] type, 平移向量t (Translation vector).
%    scale - [1,1] size,[double] type, 用于归一化图像坐标的缩放因子 (Scale factor for normalizing the image coordinates).
%
% Outputs:
%    xy - [m,2] size,[double] type, 世界坐标 (World coordinates) corresponding to the input image coordinates.
%
% Example:
%    None
% 
% See also: PlaneProjectorForwardFcn
%
% References:
%     https://github.com/opencv/opencv/blob/5c6c6af4ec1f35e1c20f366258793f711e428c1a/modules/stitching/include/opencv2/stitching/detail/warpers_inl.hpp#L237

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
    t (:,3) double = zeros(size(uv,1),3)
    scale (1,1) double =mean([K(1,1),K(2,2)])
end

% scale = mean([K(1,1),K(2,2)]);
u = uv(:,1)./scale - t(:,1);
v = uv(:,2)./scale - t(:,2);

xyz = K*R'*[u';v';ones(1,length(u))-t(:,3)'];
xyz = xyz./xyz(3,:);
xy = xyz(1:2,:)';
end


