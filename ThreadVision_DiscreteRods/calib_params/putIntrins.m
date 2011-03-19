function putIntrins( camNum, fc_in, cc_in, kc_in)
%format long
file = fopen(['intrinsics.basic_params.cam' num2str(camNum) '.txt'], 'w');
fprintf(file, [num2str(fc_in(1),20) '   ' num2str(fc_in(2),20) '\n']);
fprintf(file, [num2str(cc_in(1)-1.0,20) '   ' num2str(cc_in(2)-1.0,20) '\n']);
fprintf(file, [num2str(kc_in(1),20) '   ' num2str(kc_in(2),20) '   ' num2str(kc_in(3),20) '   ' num2str(kc_in(4),20) '   ' num2str(kc_in(5),20)]);
format short
end

