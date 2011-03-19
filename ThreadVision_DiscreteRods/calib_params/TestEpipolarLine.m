rot1 = (textRead('extrinsics.rotationVector.cam1.optimized.txt'))';
rot2 = (textRead('extrinsics.rotationVector.cam2.optimized.txt'))';
rot3 = (textRead('extrinsics.rotationVector.cam3.optimized.txt'))';
trans1 = (textRead('extrinsics.translationVector.cam1.optimized.txt'))';
trans2 = (textRead('extrinsics.translationVector.cam2.optimized.txt'))';
trans3 = (textRead('extrinsics.translationVector.cam3.optimized.txt'))';

A = textread('intrinsics.basic_params.cam1.txt');
focLength1 = (A(1,1:2))';
princPoint1 = (A(2,1:2))';
distortions1 = (A(3,1:4))';

A = textread('intrinsics.basic_params.cam2.txt');
focLength2 = (A(1,1:2))';
princPoint2 = (A(2,1:2))';
distortions2 = (A(3,1:4))';

A = textread('intrinsics.basic_params.cam3.txt');
focLength3 = (A(1,1:2))';
princPoint3 = (A(2,1:2))';
distortions3 = (A(3,1:4))';


param1 = [focLength1; princPoint1; trans1; rot1];
param2 = [focLength2; princPoint2; trans2; rot2];
param3 = [focLength3; princPoint3; trans3; rot3];


im1 = imread('extrins1-1.tif');
im2 = imread('extrins2-1.tif');
im3 = imread('extrins3-1.tif');



row = 559;
col = 455;

[m1,b1] = FindEpipolarLine(param2,param1,[row;col]);
[m3,b3] = FindEpipolarLine(param2,param3,[row;col]);

% for r=row-1:row+1
%     for c=col-1:col+1
%         im1(r,c,1) = 200;
%         im1(r,c,2) = 0;
%         im1(r,c,3) = 0;
%     end
% end



for c=1:size(im1,2)
    if (c == floor(size(im1,2)/2))
        [m2,b2] = FindEpipolarLine(param1, param2,[m1*floor(size(im1,2)/2)+b1;floor(size(im1,2)/2)]);
    end
    r = round(m1*c+b1);
    if r>0 && r<= size(im1,1)
        im1(r,c,1) = 200;
        im1(r,c,2) = 0;
        im1(r,c,3) = 0;
    end
end


for c=1:size(im3,2)
    if (c == floor(size(im3,2)/2))
        [m22,b22] = FindEpipolarLine(param3,param2,[m3*floor(size(im3,2)/2)+b3;floor(size(im3,2)/2)]);
    end
    r = round(m3*c+b3);
    if r>0 && r<= size(im3,1)
        im3(r,c,1) = 0;
        im3(r,c,2) = 0;
        im3(r,c,3) = 200;
    end
end



for c=1:size(im1,2)
    r = round(m2*c+b2);
    if r>0 && r<= size(im1,1)
        im2(r,c,1) = 200;
        im2(r,c,2) = 0;
        im2(r,c,3) = 0;
    end
    r = round(m22*c+b22);
    if r>0 && r<= size(im1,1)
        im2(r,c,1) = 0;
        im2(r,c,2) = 0;
        im2(r,c,3) = 200;
    end
end

figure; imshow(im1);
figure; imshow(im2);
figure; imshow(im3);