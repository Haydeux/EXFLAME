% Assuming stereoParams is your stereoParameters object
% Extract relevant parameters
intrinsics_left = stereoParams.CameraParameters1.K; %intrinsics_left = stereoParams.CameraParameters1.IntrinsicMatrix;
intrinsics_right = stereoParams.CameraParameters2.K; %intrinsics_right = stereoParams.CameraParameters2.IntrinsicMatrix;
extrinsics_R = stereoParams.PoseCamera2.R; %extrinsics_R = stereoParams.RotationOfCamera2;
extrinsics_T = stereoParams.PoseCamera2.Translation; %extrinsics_T = stereoParams.TranslationOfCamera2;

rad_dist_left = stereoParams.CameraParameters1.RadialDistortion;
tang_dist_left = stereoParams.CameraParameters1.TangentialDistortion;
rad_dist_right = stereoParams.CameraParameters2.RadialDistortion;
tang_dist_right = stereoParams.CameraParameters1.TangentialDistortion;


intrinsics_left = [intrinsics_left,[0;0;0],[0;0;0]];
intrinsics_right = [intrinsics_right,[0;0;0],[0;0;0]];
dist_left = [rad_dist_left, tang_dist_left, 0];
dist_right = [rad_dist_right, tang_dist_right, 0];
extrinsics_R = [extrinsics_R,[0;0;0],[0;0;0]];
extrinsics_T = [extrinsics_T,0,0];

% Save to a text file
%dlmwrite('calibration_results.txt', [intrinsics_left; intrinsics_right; extrinsics_R; extrinsics_T]);

writematrix(intrinsics_left, "stereoParams.txt");
writematrix(dist_left, "stereoParams.txt", "WriteMode", "append");
writematrix(intrinsics_right, "stereoParams.txt", "WriteMode", "append");
writematrix(dist_right, "stereoParams.txt", "WriteMode", "append");
writematrix(extrinsics_R, "stereoParams.txt", "WriteMode", "append");
writematrix(extrinsics_T, "stereoParams.txt", "WriteMode", "append");
