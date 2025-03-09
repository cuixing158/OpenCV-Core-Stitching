function [K1_opt,R1_opt,K2_opt,R2_opt] = bundleAdjustmentKR(K1,R1,K2,R2,inlierMatchedPoints1,inlierMatchedPoints2)
% Brief: BA优化匹配的两张图像对应相机内参K，旋转矩阵R，仅对相机只有旋转有效
% Details:
%    根据两幅图像匹配的inlierMatchedPoints1,inlierMatchedPoints2二位点集和初始化的内参K1,K2，旋转矩阵R1,R2，BA优化返回最佳参数，
% K1，K2只优化焦距，R1,R2转换为旋转向量3个独立自由度，共4个参数用于BA优化。
%
% Syntax:
%     [K1,R1,K2,R2] = bundleAdjustmentKR(K1,R1,K2,R2,inlierMatchedPoints1,inlierMatchedPoints2)
%
% Inputs:
%    K1 - [3,3] size,[double]
%    type,第一幅图像对应的相机初始内参矩阵K，形如[fx1,0,cx1;0,fy1,cy1;0,0,1]
%    R1 - [3,3] size,[double] type,第一幅图像对应的相机的初始旋转矩阵
%    K2 - [3,3] size,[double] type,
%    type,第二幅图像对应的相机初始内参矩阵K，形如[fx2,0,cx2;0,fy2,cy2;0,0,1]
%    R2 - [3,3] size,[double] type,第二幅图像对应的相机的初始旋转矩阵
%    inlierMatchedPoints1 - [m,2] size,[double] type,第一幅图像的匹配点集，每行形如[x,y]
%    inlierMatchedPoints2 - [m,2] size,[double] type,第二幅图像的匹配点集，每行形如[x,y]
%
% Outputs:
%    K1 - [3,3] size,[double] type,第一幅图像对应的优化后相机内参矩阵K，形如[fx1,0,cx1;0,fy1,cy1;0,0,1]
%    R1 - [3,3] size,[double] type,第一幅图像对应的优化后相机的旋转矩阵
%    K2 - [3,3] size,[double] type,第一幅图像对应的优化后相机内参矩阵K，形如[fx2,0,cx2;0,fy2,cy2;0,0,1]
%    R2 - [3,3] size,[double] type,第二幅图像对应的优化后相机的旋转矩阵
%
% Example:
%    None
%
% Reference:
%    BA优化目标函数，采用重投影误差
%    https://github.com/opencv/opencv/blob/57a78cb9dfd591c95efe4afd2953db97993d3702/modules/stitching/src/motion_estimators.cpp#L382
%
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         06-Mar-2025 17:11:58
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
    K1 (3,3) double
    R1 (3,3) double
    K2 (3,3) double
    R2 (3,3) double
    inlierMatchedPoints1 (:,2) double
    inlierMatchedPoints2 (:,2) double
end

% 将初始旋转矩阵转换为旋转向量
rvec1 = rotationMatrixToVector(R1);
rvec2 = rotationMatrixToVector(R2);

% 初始参数：焦距和旋转向量
params0 = [K1(1,1); rvec1(:); K2(1,1); rvec2(:)];

% 优化配置
options = optimoptions('lsqnonlin',...
    'Algorithm','levenberg-marquardt',...
    'Display','off',...
    'MaxIterations',200,...
    'FunctionTolerance',1e-8,...
    'StepTolerance',1e-8);

% 参数边界
lb = [log(0.1*K1(1,1)); -pi*ones(3,1); log(0.1*K2(1,1)); -pi*ones(3,1)];
ub = inf*ones(size(lb));

% 执行优化
params_opt = lsqnonlin(@(p) errorFunction(p, K1, K2, inlierMatchedPoints1, inlierMatchedPoints2), params0, lb, ub, options);
% fprintf("exitflag:%d\n", exitflag);

% 结果提取与转换
f1_opt = params_opt(1);
rvec1_opt = params_opt(2:4);
f2_opt = params_opt(5);
rvec2_opt = params_opt(6:8);

% 构建优化后的内参矩阵（仅更新焦距，其余参数保持不变）
K1_opt = K1;
K1_opt(1,1) = f1_opt;
K1_opt(2,2) = f1_opt;

K2_opt = K2;
K2_opt(1,1) = f2_opt;
K2_opt(2,2) = f2_opt;

% 旋转向量转换为旋转矩阵
R1_opt = rotvec2mat3d(rvec1_opt);
R2_opt = rotvec2mat3d(rvec2_opt);

end

%% 误差函数,重投影误差
function err = errorFunction(params, K1, K2, pts1, pts2)
    % 参数解析
    f1 = params(1);     % 相机1焦距
    rvec1 = params(2:4); % 相机1旋转向量
    f2 = params(5);     % 相机2焦距
    rvec2 = params(6:8); % 相机2旋转向量
    
    % 从初始K矩阵提取固定参数（假设主点和纵横比不参与优化）
    a1 = K1(2,2)/K1(1,1);   % 纵横比 fy/fx
    ppx1 = K1(1,3);         % 主点x
    ppy1 = K1(2,3);         % 主点y
    a2 = K2(2,2)/K2(1,1);
    ppx2 = K2(1,3);
    ppy2 = K2(2,3);

    % 重建带纵横比的内参矩阵
    K1_adj = [f1, 0,   ppx1;
              0,  f1*a1, ppy1;
              0,  0,    1];
    K2_adj = [f2, 0,   ppx2;
              0,  f2*a2, ppy2;
              0,  0,    1];
    
    % 计算旋转矩阵（保证正交性）
    R1 = rotationVectorToMatrix(rvec1);
    [U,~,V] = svd(R1);
    R1 = U*V';
    
    R2 = rotationVectorToMatrix(rvec2);
    [U,~,V] = svd(R2);
    R2 = U*V';
    
    % 构建单应矩阵 H = K2*R2^{-1}*R1*K1^{-1}
    H = K2_adj / R2 * R1 / K1_adj; % 等价于K2_adj*inv(R2)*R1*inv(K1_adj)
    
    % 计算重投影误差
    num_points = size(pts1,1);
    err = zeros(num_points*2,1); % 每个点贡献x,y两个误差
    
    for i = 1:num_points
        p1 = [pts1(i,1), pts1(i,2), 1]'; % 齐次坐标
        
        % 单应变换
        p_proj = H * p1;
        x_proj = p_proj(1)/p_proj(3); % 归一化坐标
        y_proj = p_proj(2)/p_proj(3);
        
        % 实际观测坐标
        p2 = pts2(i,:);
        
        % 计算重投影误差
        err(2*i-1) = p2(1) - x_proj;
        err(2*i)   = p2(2) - y_proj;
    end
end


function R = rotationVectorToMatrix(rvec)
% 将旋转向量转换为旋转矩阵
% 输入：
%   rvec - 3x1 或 1x3 旋转向量 [rx; ry; rz]
% 输出：
%   R - 3x3 旋转矩阵

% 确保输入为列向量
rvec = rvec(:);

% 计算旋转角度 theta
theta = norm(rvec);

if theta < eps
    % 如果旋转角度接近0，返回单位矩阵
    R = eye(3);
else
    % 计算旋转轴单位向量
    k = rvec / theta;
    
    % 构造叉积矩阵
    K = [0    -k(3)  k(2);
         k(3)  0    -k(1);
        -k(2)  k(1)  0];
    
    % 使用Rodrigues公式计算旋转矩阵
    R = eye(3) + sin(theta)*K + (1-cos(theta))*(K^2);
end
end

function rvec = rotationMatrixToVector(R)
    theta = acos((trace(R)-1)/2);
    if abs(theta) < eps
        rvec = zeros(3,1);
    else
        axis = 1/(2*sin(theta)) * [R(3,2)-R(2,3); 
                                  R(1,3)-R(3,1);
                                  R(2,1)-R(1,2)];
        rvec = theta * axis;
    end
    rvec = rvec(:)';
end



