%%% INPUT THE IMAGE FILE NAME:
for camNum=1:3
    nums = 400:499;
    pre = 'knottie';
    directory = [RemoteHomeDirectory '/data/2010Jul23/'];
    load([directory 'calib/calib' num2str(camNum) '.mat']);
    KK = [fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2) ; 0 0 1];
    
    for imNum = nums
        %% UNDISTORT THE IMAGE:
        filename = [directory pre num2str(camNum) '-' num2str(imNum) '.tiff']
        I = imread(filename);
        fprintf(1,'Computing the undistorted image...')

        [Ipart_1] = rect(im2double(I(:,:,1)),eye(3),fc,cc,kc,alpha_c,KK);
        [Ipart_2] = rect(im2double(I(:,:,2)),eye(3),fc,cc,kc,alpha_c,KK);
        [Ipart_3] = rect(im2double(I(:,:,3)),eye(3),fc,cc,kc,alpha_c,KK);

        I2 = ones(ny, nx,3);
        I2(:,:,1) = Ipart_1;
        I2(:,:,2) = Ipart_2;
        I2(:,:,3) = Ipart_3;

        %% SAVE THE IMAGE IN FILE:
        outpath = [directory '/rect_color/' pre num2str(camNum) '-_rect_color' num2str(imNum) '.tif']
        fprintf(1,['Saving undistorted image under ' outpath '...'])
        imwrite(im2uint8(I2),outpath,'TIFF');
        fprintf(1,'done\n')
    end
    %clear
end