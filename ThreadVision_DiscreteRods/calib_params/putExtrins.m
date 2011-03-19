function putExtrins( camNum, Tc_ext, omc_ext)
format long
filet = fopen(['extrinsics.translationVector.cam' num2str(camNum) '.optimized.txt'], 'w');
filer = fopen(['extrinsics.rotationVector.cam' num2str(camNum) '.optimized.txt'], 'w');
fprintf(filet, [num2str(Tc_ext(1),20) '   ' num2str(Tc_ext(2),20) '   ' num2str(Tc_ext(3),20)]);
fprintf(filer, [num2str(omc_ext(1),20) '   ' num2str(omc_ext(2),20) '   ' num2str(omc_ext(3),20)]);
format short
end

