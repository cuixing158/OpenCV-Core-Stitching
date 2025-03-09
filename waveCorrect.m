function rotM = waveCorrect(rotations,options)
% Brief: 执行与OpenCV stitching模块中的wareCorrect函数一样的功能
% Details:
%    根据输入的多个旋转矩阵rotations和对应可选的修正类型["horization","vertical"]之一，修正各个旋转矩阵
% 
% Syntax:  
%     rotM = waveCorrect(rotations)
%     rotM = waveCorrect(rotations,Type="horization")
%     rotM = waveCorrect(rotations,Type="vertical")
% 
% Inputs:
%    rotations - [3,3,m] size,[double] type,m个3x3旋转矩阵
%    options - [1,1] size,name-value键值对，用于选择类型
% 
% Outputs:
%    rotM - [3,3,m] size,[double] type,m个3x3旋转矩阵
% 
% Example: 
%    None
% Reference:
%    https://github.com/opencv/opencv/blob/dbd4e4549d00a4b455058adb31dcf8630499cdee/modules/stitching/src/motion_estimators.cpp#L932
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         06-Mar-2025 09:59:21
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
    rotations (3,3,:) double
    options.Type (1,1) string {mustBeMember(options.Type,["horization","vertical"])}="horization"
end

m = size(rotations,3);
rotM = rotations;

if m <= 1
    return;
end

%% 1. 计算moment矩阵
moment = zeros(3);
for i = 1:m
    col = rotM(:,1,i); % 注意MATLAB是列存储
    moment = moment + col * col';
end

%% 2. 特征分解
[V,D] = eig(moment);
eigen_vals = diag(D);
[~,idx] = sort(eigen_vals,'descend'); % 确保特征值降序排列
eigen_vecs = V(:,idx); % 对应列向量

%% 3. 选择基向量
if options.Type == "horization"
    rg1 = eigen_vecs(:,3); % 第三大特征值对应列向量
elseif options.Type == "vertical"
    rg1 = eigen_vecs(:,1); % 最大特征值对应列向量
end

%% 4. 计算正交基
img_k = zeros(3,1);
for i = 1:m
    img_k = img_k + rotM(:,3,i); % 累加第三列
end
rg0 = cross(rg1, img_k);
rg0_norm = norm(rg0);

if rg0_norm <= eps
    return;
end
rg0 = rg0 / rg0_norm;
rg2 = cross(rg0, rg1);

%% 5. 方向一致性校正
conf = 0;
if options.Type == "horization"
    for i = 1:m
        conf = conf + dot(rg0, rotM(:,1,i));
    end
    if conf < 0
        rg0 = -rg0;
        rg1 = -rg1;
    end
elseif options.Type == "vertical"
    for i = 1:m
        conf = conf - dot(rg1, rotM(:,1,i));
    end
    if conf < 0
        rg0 = -rg0;
        rg1 = -rg1;
    end
end

%% 6. 构建校正矩阵
R = [rg0';   % 第一行
     rg1';    % 第二行
     rg2'];   % 第三行

%% 7. 应用校正
for i = 1:m
    rotM(:,:,i) = R * rotM(:,:,i); % 矩阵乘法
end

end