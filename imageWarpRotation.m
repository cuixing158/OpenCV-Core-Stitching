function [outImage,mapX,mapY,dst_tl,dst_br] = imageWarpRotation(inImage,K,R,options)
% Brief: 
%    This function performs image warping based on camera intrinsic parameters and rotation matrix.
%
% Details:
%    The function takes an input image with camera intrinsic matrix K (3x3) and a rotation matrix R (3x3),
%    and applies a projection transformation to the input image. The type of projection can be specified
%    through the options parameter, supporting "plane", "spherical", "cylindrical", "fisheye", and "stereo".
%    The function returns the transformed output image, along with the x and y mapping arrays, and the 
%    top-left and bottom-right coordinates of the transformed image relative to the input image coordinate system.
%
% Syntax:  
%     [outImage,mapX,mapY,dst_tl,dst_br] = imageWarpRotation(inImage,K,R,options)
% 
% Inputs:
%    inImage - [m,n] size, Input image array
%    K - [3,3] size,[double] type, Camera intrinsic matrix, in the form of [fx,0,cx;0,fy,cy;0,0,1]
%    R - [3,3] size,[double] type, Relative rotation matrix
%    options - [1,1] size, Name-value pair arguments, supporting the selection of projection type. Currently supports "plane", "spherical", "cylindrical", "fisheye", and "stereo".
% 
% Outputs:
%    outImage - [om,on] size, Output destination image
%    mapX - [om,on] size,[double] type, x mapping array
%    mapY - [om,on] size,[double] type, y mapping array
%    dst_tl - [1,2] size,[double] type, Position of the top-left corner of the transformed output image relative to the input image coordinate system
%    dst_br - [1,2] size,[double] type, Position of the bottom-right corner of the transformed output image relative to the input image coordinate system
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         05-Mar-2025 14:37:33
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024b
% Copyright © 2025 TheMatrix.All Rights Reserved.
%

arguments
    inImage 
    K (3,3) double
    R (3,3) double
    options.Scale (1,1) double = mean([K(1,1),K(2,2)]) 
    options.WarperType (1,1) string {mustBeMember(options.WarperType,["plane","spherical","cylindrical","fisheye","stereo"])} = "plane"
    options.FillValues (1,1) double {mustBeInRange(options.FillValues,0,255)} = 0
    options.BorderMode (1,1) {mustBeMember(options.BorderMode,...
        [ "BORDER_CONSTANT",... % !< `000000|abcdefgh|00000`  with specified `0`
        "BORDER_REPLICATE",... % !< `aaaaaa|abcdefgh|hhhhhhh`
        "BORDER_REFLECT",... % !< `fedcba|abcdefgh|hgfedcb`
        ])} = "BORDER_CONSTANT"
end

scale = options.Scale;
[h,w,~] = size(inImage);


[X,Y] = meshgrid(1:w,1:h);
cc = [X(:),Y(:)];
t = zeros(size(cc,1),3);

if options.WarperType=="plane"
    dstcc = PlaneProjectorForwardFcn(cc,K,R,t,scale);
elseif options.WarperType =="spherical"
    dstcc = SphericalProjectorForwardFcn(cc,K,R,scale);
elseif options.WarperType =="cylindrical"
    dstcc = CylindricalProjectorForwardFcn(cc,K,R,scale);
elseif options.WarperType =="fisheye"
    dstcc = FisheyeProjectorForwardFcn(cc,K,R,scale);
elseif options.WarperType =="stereo"
    dstcc = StereoProjectorForwardFcn(cc,K,R,scale);
end

dst_tl = round([min(dstcc(:,1)),min(dstcc(:,2))]);
dst_br = round([max(dstcc(:,1)),max(dstcc(:,2))]);

xlimitRange = [dst_tl(1),dst_br(1)];
ylimitRange = [dst_tl(2),dst_br(2)];
outWidth = diff(xlimitRange)+1;
outHeight = diff(ylimitRange)+1;

[X,Y] = meshgrid(xlimitRange(1):xlimitRange(2),ylimitRange(1):ylimitRange(2));
cc = [X(:),Y(:)];
t = zeros(size(cc,1),3);

if options.WarperType=="plane"
    outCC = PlaneProjectorInverseFcn(cc,K,R,t,scale);
elseif options.WarperType =="spherical"
    outCC = SphericalProjectorInverseFcn(cc,K,R,scale);
elseif options.WarperType =="cylindrical"
    outCC = CylindricalProjectorInverseFcn(cc,K,R,scale);
elseif options.WarperType =="fisheye"
    outCC = FisheyeProjectorInverseFcn(cc,K,R,scale);
elseif options.WarperType =="stereo"
    outCC = StereoProjectorInverseFcn(cc,K,R,scale);
end

mapX = reshape(outCC(:,1),outHeight,outWidth);
mapY = reshape(outCC(:,2),outHeight,outWidth);

outImage = imageInterp(inImage,mapX,mapY,FillValues=options.FillValues,BorderMode=options.BorderMode);
end
