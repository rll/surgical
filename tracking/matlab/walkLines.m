% dbstop if error
set(0,'DefaultFigureWindowStyle','normal');
opt = 'initialmagnification'

directory = HomeDirectory;
imDir = 'data/2010Jul01';
pre = 'test';

nx = 1280;
ny = 960;
xlimls = [-1 -1 -1];
xlimrs = [985 nx+1 1031]; 
ylimts = [212 220 -1];

close all
searchSide = 150;
maskSide = 350;
cropSide = 250;
side = 'left';

nums = [36:71 84:92 305:337 406:492 569:579];
startpts = [ [824;281] [681;217] [480;182] [483;220] [523;203]];
starti = [36 84 305 406 569];

ex_loc = [561;338];        

for k=1:1
    %% iter
    
    nums2 = 569:579;
    locs = zeros(2, length(nums2));

    for i2=1:length(nums2);
        %% iter
        imNum = nums2(i2);
        if ismember(imNum, starti)
            ix = find(starti == imNum);
            ex_loc = startpts(:,ix)
        end

        getHoughLines
        
        doplot = false;        
        if doplot
            figure; imshow(bw2, 'initialmagnification', 30); hold on; 
            showhoughsegs
        end
        
        % walk along the bottom lines, look above
        % get bottom lines
        xbase = 100000;
        for i=1:length(lines)
            x = lines(i).point1(1);
            if x < xbase 
                xbase = x;
            end
        end
        
        icepts = zeros(1,length(lines));
        for i=1:length(lines)
            m = (lines(i).point2(2) - lines(i).point1(2))/(lines(i).point2(1) - lines(i).point1(1));
            icepts(i) = lines(i).point1(2) - (lines(i).point1(1) - xbase)*m;
        end
        
        [icepts,I] = sort(icepts);
        icdiffs = icepts(2:end) - icepts(1:end-1);
        [gap,i] = max(icdiffs);
        
        if gap > 10
            tlines = lines(I(1:i));
            blines = lines(I(i+1:end));
        else
            blines = lines;
        end
        armw = 40;
        dispim = im;
        thislocs = zeros(2,0);
        gradStrengths = zeros(1,0);

        if doplot
            linesbak = lines;
            lines = blines;
            figure; imshow(bw2, 'initialmagnification', 30); hold on;
            showhoughsegs
            lines = linesbak;        
        end
        for i=1:length(blines)
            %% iter
            rawxx = 1:norm(blines(i).point2 - blines(i).point1);
            rawyy = -1:-1:-armw;
            rise = blines(i).point2(2) - blines(i).point1(2);
            run = blines(i).point2(1) - blines(i).point1(1);
            hyp = norm(blines(i).point2 - blines(i).point1);
            
            c = run/hyp;
            s = rise/hyp;
            
            R = [c -s; s c];
            
            vals = zeros(length(rawyy), length(rawxx));
            imgsm = conv2(img, fspecial('gaussian', [9,9],2), 'same');
            
            for j=1:length(rawyy)
                y = rawyy(j);
                X = [rawxx; ones(1,length(rawxx))*y];
                tpts = R*X;
                tpts = vadd(tpts, blines(i).point1);                
                xx = tpts(1,:);
                yy = tpts(2,:);
                xx = round(xx);
                yy = round(yy);
                bool = xx > 1 & xx <= nx & yy > 1 & yy <= ny;
                xx = xx(bool);
                yy = yy(bool);
                
                vs = imgsm(sub2ind(size(imgsm), round(yy),round(xx))); 
                vals(j,bool) = vs;
            end
            
            vdiffs = vals(:, 2:end) - vals(:,1:end-1);
            stvd = std(vdiffs, 1);
            
            
            mvals = mean(vals,1);
            mvdiffs = mvals(2:end) - mvals(1:end-1);
%             mvdiffs2 = conv(double(mvdiffs), normpdf(-4:4, 0, 1), 'same');
            mvdiffs2 = mvdiffs;
            gradthresh = 2.5;
            devthresh  = 5;
            dev = (gradthresh - mvdiffs2 - stvd) ;
            
            doplot3 = false;
            if doplot3
                figure('name', [num2str(imNum), ' ', num2str(i)]);
                plot(mvdiffs2); hold on;
                plot([1 length(mvdiffs2)], [gradthresh gradthresh], 'g');
                plot(dev, 'r');
                plot([1 length(mvdiffs2)], [devthresh devthresh], 'm');
            end
            
            lin = vadd(R*[rawxx; zeros(1,length(rawxx))], blines(i).point1);
            dists = normArray(vadd(lin, -ex_loc));
            inds = find(mvdiffs2 > gradthresh & dists(1:end-1) < 50 & dev < devthresh);

            if isempty(inds)% && min(dists) >= 50
                inds = find(mvdiffs2 > gradthresh & dev < devthresh);
            end 
               
            for i3=1:length(inds)
                ind = inds(i3);
                mvd = mvdiffs2(ind);
                mvd, ind
                xy = [rawxx(ind); 0];
                txy = R*xy + blines(i).point1';
                
                thislocs = [thislocs txy];
                gradStrengths = [gradStrengths mvd];
                x = round(txy(1));
                y = round(txy(2));
                dispim(y, x,2) = 255;
                dispim(y, x+1,2) = 255;
                dispim(y, x-1,2) = 255;
                dispim(y+1, x,2) = 255;
                dispim(y-1, x,2) = 255;
%                 figure(fig); hold on; scatter(txy(1), y);
            end            
                        
        end
        
        % choose a location based gradient strength and proximity to
        % expected
        distPart = normArray(vadd(thislocs, -ex_loc));
        gradPart = gradStrengths;
        scores = gradPart; %./distPart;
        [mx, besti] = max(scores);        
        loc = thislocs(:,besti);
        
        if ~isempty(thislocs)
            locs(:, i2) = loc;
            [mx, distPart(besti), gradPart(besti)]
            loc
            ex_loc = loc;

            x = round(loc(1)); y = round(loc(2));
            dispim(y, x,3) = 255;
            dispim(y, x+1,3) = 255;
            dispim(y, x-1,3) = 255;
            dispim(y+1, x,3) = 255;
            dispim(y-1, x,3) = 255;
            dispim(y, x,2) = 0;
            dispim(y, x+1,2) = 0;
            dispim(y, x-1,2) = 0;
            dispim(y+1, x,2) = 0;
            dispim(y-1, x,2) = 0;

            x0 = max(1, min(thislocs(1,:)) - 100);
            x1 = min(nx, max(thislocs(1,:)) + 100);
            y0 = max(1, min(thislocs(2,:)) - 100);
            y1 = min(ny, max(thislocs(2,:)) + 100);
        end
        
        cropped = dispim(y0:y1, x0:x1,:);
        doplot2 = true;
        if doplot2
            figure; imshow(cropped);
        end
        
        imwrite(cropped, ...
            [RemoteHomeDirectory 'results/band/band' num2str(k) ...
            '-' num2str(imNum) '.tiff']);

    end
end
