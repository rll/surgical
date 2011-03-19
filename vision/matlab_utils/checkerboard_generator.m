function [im] = checkerboard_generator(pix_per_square, rows, cols)

deleteRow = false;
deleteCol = false;


if (mod(rows,2) == 1)
    deleteRow = true;
end

if (mod(cols,2) == 1)
    deleteCol = true;
end

rows = round(rows/2);
cols = round(cols/2);

im = checkerboard(pix_per_square, rows, cols);
im(find(im < 0.9 & im >0.1)) = 1.0;

if (deleteRow)
    im = im(1:end-pix_per_square,:);
end

if (deleteCol)
    im = im(:,1:end-pix_per_square);
end

imwrite(im, 'checkerboard.png', 'png');



end