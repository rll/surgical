% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2022.285969507139498 ; 2022.055665742384008 ];

%-- Principal point:
cc = [ 653.296316100987156 ; 477.253121198827444 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.338700508380346 ; 0.120640626122158 ; -0.000623249962558 ; 0.000971074348488 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.103887046975689 ; 1.099983565473995 ];

%-- Principal point uncertainty:
cc_error = [ 2.245168097808697 ; 2.275410928842370 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.003059888377342 ; 0.029922084139553 ; 0.000141596621220 ; 0.000125810694890 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 960;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 29;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.579804e+00 ; -1.048894e+00 ; 9.267757e-01 ];
Tc_1  = [ -3.652052e+01 ; 4.325122e+01 ; 3.144466e+02 ];
omc_error_1 = [ 1.026853e-03 ; 7.829088e-04 ; 1.627551e-03 ];
Tc_error_1  = [ 3.493586e-01 ; 3.583951e-01 ; 1.991750e-01 ];

%-- Image #2:
omc_2 = [ 2.875210e+00 ; 4.963056e-01 ; -4.450997e-01 ];
Tc_2  = [ -5.904583e+01 ; 2.927628e+01 ; 3.245360e+02 ];
omc_error_2 = [ 1.186355e-03 ; 5.061108e-04 ; 1.840770e-03 ];
Tc_error_2  = [ 3.610112e-01 ; 3.672711e-01 ; 1.935162e-01 ];

%-- Image #3:
omc_3 = [ 2.353735e+00 ; 6.011789e-02 ; 5.174462e-01 ];
Tc_3  = [ 1.674939e+01 ; 1.306873e+00 ; 3.224989e+02 ];
omc_error_3 = [ 1.184209e-03 ; 6.731536e-04 ; 1.403166e-03 ];
Tc_error_3  = [ 3.586219e-01 ; 3.632062e-01 ; 2.142020e-01 ];

%-- Image #4:
omc_4 = [ 2.355687e+00 ; 1.640246e-01 ; 5.070398e-01 ];
Tc_4  = [ -9.261206e+01 ; 4.304398e+00 ; 3.167324e+02 ];
omc_error_4 = [ 1.151155e-03 ; 6.054437e-04 ; 1.365176e-03 ];
Tc_error_4  = [ 3.524435e-01 ; 3.656306e-01 ; 2.403454e-01 ];

%-- Image #5:
omc_5 = [ 2.282433e+00 ; -7.140278e-02 ; 2.245904e-01 ];
Tc_5  = [ -8.603971e+01 ; 3.892406e+01 ; 3.441713e+02 ];
omc_error_5 = [ 1.131199e-03 ; 5.947766e-04 ; 1.326999e-03 ];
Tc_error_5  = [ 3.835793e-01 ; 3.973772e-01 ; 2.539057e-01 ];

%-- Image #6:
omc_6 = [ -1.634526e+00 ; -8.634147e-03 ; -1.992594e-01 ];
Tc_6  = [ -7.521245e+01 ; -3.884520e+01 ; 3.622347e+02 ];
omc_error_6 = [ 1.131441e-03 ; 8.193751e-04 ; 9.639080e-04 ];
Tc_error_6  = [ 4.039990e-01 ; 4.135327e-01 ; 2.036038e-01 ];

%-- Image #7:
omc_7 = [ 3.054211e+00 ; 2.866334e-01 ; -2.830766e-01 ];
Tc_7  = [ -5.461608e+01 ; 1.948902e+01 ; 3.276863e+02 ];
omc_error_7 = [ 1.375741e-03 ; 3.603073e-04 ; 2.223476e-03 ];
Tc_error_7  = [ 3.638249e-01 ; 3.701059e-01 ; 1.947864e-01 ];

%-- Image #8:
omc_8 = [ 2.736156e+00 ; 9.047175e-01 ; -8.872187e-01 ];
Tc_8  = [ -9.379248e+01 ; 1.283205e+01 ; 3.391319e+02 ];
omc_error_8 = [ 9.968884e-04 ; 9.355590e-04 ; 1.769454e-03 ];
Tc_error_8  = [ 3.800210e-01 ; 3.888610e-01 ; 2.051738e-01 ];

%-- Image #9:
omc_9 = [ 2.769851e+00 ; 8.585210e-01 ; -8.399687e-01 ];
Tc_9  = [ -3.921970e+01 ; 7.894809e+00 ; 3.453398e+02 ];
omc_error_9 = [ 1.042300e-03 ; 7.586951e-04 ; 1.719295e-03 ];
Tc_error_9  = [ 3.834308e-01 ; 3.878633e-01 ; 1.820550e-01 ];

%-- Image #10:
omc_10 = [ 2.914573e+00 ; -7.876977e-01 ; 8.680837e-01 ];
Tc_10  = [ 3.001522e+01 ; 5.246804e+01 ; 2.909009e+02 ];
omc_error_10 = [ 1.052435e-03 ; 8.752438e-04 ; 1.899188e-03 ];
Tc_error_10  = [ 3.247619e-01 ; 3.287157e-01 ; 1.845231e-01 ];

%-- Image #11:
omc_11 = [ -2.931226e+00 ; 7.412735e-01 ; -8.150204e-01 ];
Tc_11  = [ -3.348019e+01 ; 6.384435e+01 ; 2.761958e+02 ];
omc_error_11 = [ 1.168594e-03 ; 2.922194e-04 ; 1.804773e-03 ];
Tc_error_11  = [ 3.105054e-01 ; 3.143159e-01 ; 1.768323e-01 ];

%-- Image #12:
omc_12 = [ -2.001712e+00 ; 2.361119e-02 ; -1.533018e-01 ];
Tc_12  = [ -6.778667e+01 ; 2.551540e+01 ; 3.073145e+02 ];
omc_error_12 = [ 1.145420e-03 ; 7.245546e-04 ; 1.140449e-03 ];
Tc_error_12  = [ 3.414615e-01 ; 3.514356e-01 ; 1.696536e-01 ];

%-- Image #13:
omc_13 = [ -2.185893e+00 ; 2.261394e-02 ; -1.238872e-01 ];
Tc_13  = [ -2.457242e+01 ; 3.210161e+01 ; 2.961422e+02 ];
omc_error_13 = [ 1.162845e-03 ; 6.499795e-04 ; 1.249139e-03 ];
Tc_error_13  = [ 3.293119e-01 ; 3.337508e-01 ; 1.463399e-01 ];

%-- Image #14:
omc_14 = [ -2.191336e+00 ; 2.581740e-02 ; -1.308496e-01 ];
Tc_14  = [ -3.553705e+01 ; 1.615186e+00 ; 3.304051e+02 ];
omc_error_14 = [ 1.151681e-03 ; 5.919952e-04 ; 1.245787e-03 ];
Tc_error_14  = [ 3.661295e-01 ; 3.721323e-01 ; 1.643913e-01 ];

%-- Image #15:
omc_15 = [ 2.677871e+00 ; -6.571694e-02 ; 6.196031e-02 ];
Tc_15  = [ 3.671871e+01 ; 8.193253e+00 ; 3.709663e+02 ];
omc_error_15 = [ 1.251578e-03 ; 5.103399e-04 ; 1.793599e-03 ];
Tc_error_15  = [ 4.115509e-01 ; 4.186761e-01 ; 2.514635e-01 ];

%-- Image #16:
omc_16 = [ 2.692901e+00 ; -4.825446e-02 ; 1.907868e-02 ];
Tc_16  = [ -1.043667e+02 ; 1.033616e+01 ; 3.634142e+02 ];
omc_error_16 = [ 1.271003e-03 ; 4.720544e-04 ; 1.768777e-03 ];
Tc_error_16  = [ 4.026978e-01 ; 4.189447e-01 ; 2.707741e-01 ];

%-- Image #17:
omc_17 = [ 2.287811e+00 ; 6.681175e-01 ; 4.096021e-01 ];
Tc_17  = [ -6.968292e+00 ; 7.823790e+00 ; 3.218727e+02 ];
omc_error_17 = [ 1.192305e-03 ; 5.822935e-04 ; 1.409014e-03 ];
Tc_error_17  = [ 3.579618e-01 ; 3.610608e-01 ; 2.096869e-01 ];

%-- Image #18:
omc_18 = [ 2.296455e+00 ; 8.229979e-01 ; 3.715075e-01 ];
Tc_18  = [ -7.913447e+01 ; 6.082307e+00 ; 3.180696e+02 ];
omc_error_18 = [ 1.104923e-03 ; 5.832162e-04 ; 1.400787e-03 ];
Tc_error_18  = [ 3.559567e-01 ; 3.635566e-01 ; 2.165896e-01 ];

%-- Image #19:
omc_19 = [ -9.913876e-01 ; -2.375946e+00 ; 7.959567e-01 ];
Tc_19  = [ -5.958665e+01 ; -5.015372e+01 ; 3.522682e+02 ];
omc_error_19 = [ 9.140414e-04 ; 1.088499e-03 ; 1.507783e-03 ];
Tc_error_19  = [ 3.939726e-01 ; 4.015643e-01 ; 2.070611e-01 ];

%-- Image #20:
omc_20 = [ -9.520346e-01 ; -2.335106e+00 ; 7.800338e-01 ];
Tc_20  = [ -2.636532e+01 ; -5.466681e+01 ; 3.524276e+02 ];
omc_error_20 = [ 8.745912e-04 ; 1.054519e-03 ; 1.464951e-03 ];
Tc_error_20  = [ 3.921017e-01 ; 3.980325e-01 ; 1.997742e-01 ];

%-- Image #21:
omc_21 = [ -1.030862e+00 ; -2.256886e+00 ; 6.915892e-01 ];
Tc_21  = [ 4.561132e+01 ; -6.750172e+01 ; 3.583037e+02 ];
omc_error_21 = [ 8.080695e-04 ; 9.629045e-04 ; 1.411255e-03 ];
Tc_error_21  = [ 4.021261e-01 ; 4.043754e-01 ; 2.161861e-01 ];

%-- Image #22:
omc_22 = [ -1.579648e+00 ; -2.413386e+00 ; 5.180573e-01 ];
Tc_22  = [ -2.262211e+00 ; -7.021516e+01 ; 3.768068e+02 ];
omc_error_22 = [ 9.218221e-04 ; 9.816968e-04 ; 1.796374e-03 ];
Tc_error_22  = [ 4.210909e-01 ; 4.231119e-01 ; 2.182729e-01 ];

%-- Image #23:
omc_23 = [ -1.640946e+00 ; -2.367096e+00 ; 5.218876e-01 ];
Tc_23  = [ 2.511143e+01 ; -7.668610e+01 ; 3.808683e+02 ];
omc_error_23 = [ 9.641948e-04 ; 9.968491e-04 ; 1.851991e-03 ];
Tc_error_23  = [ 4.278532e-01 ; 4.272349e-01 ; 2.288766e-01 ];

%-- Image #24:
omc_24 = [ 2.215656e+00 ; 1.472764e+00 ; 4.419091e-01 ];
Tc_24  = [ -6.456626e+01 ; -3.257929e+01 ; 3.121462e+02 ];
omc_error_24 = [ 1.095471e-03 ; 6.737419e-04 ; 1.557909e-03 ];
Tc_error_24  = [ 3.493526e-01 ; 3.539569e-01 ; 2.102685e-01 ];

%-- Image #25:
omc_25 = [ 2.239364e+00 ; 2.162764e+00 ; 1.945936e-01 ];
Tc_25  = [ -4.532315e+01 ; -6.664965e+01 ; 3.477020e+02 ];
omc_error_25 = [ 1.322777e-03 ; 1.319492e-03 ; 2.795268e-03 ];
Tc_error_25  = [ 3.901959e-01 ; 3.932400e-01 ; 2.411618e-01 ];

%-- Image #26:
omc_26 = [ 1.522391e+00 ; 2.316774e+00 ; -8.150462e-01 ];
Tc_26  = [ 2.667725e+00 ; -3.283219e+01 ; 4.069033e+02 ];
omc_error_26 = [ 6.002174e-04 ; 1.161300e-03 ; 1.611367e-03 ];
Tc_error_26  = [ 4.513330e-01 ; 4.560958e-01 ; 2.196549e-01 ];

%-- Image #27:
omc_27 = [ 2.082905e+00 ; 1.492600e+00 ; -3.507356e-01 ];
Tc_27  = [ -7.977552e+01 ; -1.406342e+01 ; 3.837427e+02 ];
omc_error_27 = [ 8.869518e-04 ; 9.037230e-04 ; 1.506895e-03 ];
Tc_error_27  = [ 4.264814e-01 ; 4.353527e-01 ; 2.362683e-01 ];

%-- Image #28:
omc_28 = [ 1.883743e+00 ; 1.921146e+00 ; -5.365826e-01 ];
Tc_28  = [ -2.868254e+01 ; -8.423754e-02 ; 3.634238e+02 ];
omc_error_28 = [ 7.796507e-04 ; 9.424810e-04 ; 1.547343e-03 ];
Tc_error_28  = [ 4.028095e-01 ; 4.079817e-01 ; 2.013548e-01 ];

%-- Image #29:
omc_29 = [ 2.195913e+00 ; 6.739950e-01 ; -2.848310e-01 ];
Tc_29  = [ -6.985832e+01 ; -2.446899e+01 ; 4.365207e+02 ];
omc_error_29 = [ 1.103674e-03 ; 7.810283e-04 ; 1.350193e-03 ];
Tc_error_29  = [ 4.853021e-01 ; 4.930709e-01 ; 2.791515e-01 ];
