function R2 = getRotationFromH(intrinsics1,intrinsics2,H,R1)
% Brief: 单应矩阵（透视变换矩阵）分解得到旋转矩阵，仅只有旋转自由度有效
% Details:
%    利用已知的相邻2幅图像相机内参intrinsics1,intrinsics2和单应性 H（匹配对中估计的单应性矩阵），通过公式
% R_rel = K_from⁻¹ · H⁻¹ · K_to 计算第2幅图像相对第1幅图像旋转矩阵R_rel
% 
% Syntax:  
%     R2 = getRotationFromH(intrinsics1,intrinsics2,H,R1)
% 
% Inputs:
%    intrinsics1 - [1,1] size,[intrinsics] type,built-in cameraIntrinsics
%    object
%    intrinsics2 - [m,n] size,[intrinsics] type,built-in cameraIntrinsics
%    object
%    H - [3,3] size,[double] type,图像1到图像2的单应矩阵（透视变换矩阵）
%    R1 - [3,3] size,[double] type,第一幅图像的旋转矩阵，一般为单位矩阵
% 
% Outputs:
%    R2 - [3,3] size,[double] type,第二幅图像的旋转矩阵
% 
% Example: 
%    None
% 
% Reference:
%   https://github.com/opencv/opencv/blob/dbd4e4549d00a4b455058adb31dcf8630499cdee/modules/stitching/src/motion_estimators.cpp#L63
%
% See also: cameraIntrinsics

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         06-Mar-2025 14:45:38
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
    intrinsics1 (1,1) cameraIntrinsics
    intrinsics2 (1,1) cameraIntrinsics
    H (3,3) double
    R1 (3,3) double = eye(3)
end

K_from = intrinsics1.K;
K_to = intrinsics2.K;
estimateR = inv(K_from)*inv(H)*K_to;
scale = nthroot(det(estimateR), 3);% 正则化使其行列式为1
R_rel = estimateR/scale;
R2 = R1*R_rel;
end