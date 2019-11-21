% read parameters from .mat files(left pair and right pair cameras) 
left_params = load('Parameters.mat');
right_params = load('Parameters.mat');
left_stereo = left_params.stereoParams;
right_stereo = right_params.stereoParams;
fileID = fopen('build/calib_params.yml', 'w');
% write header
fprintf(fileID, "%s\n%s\n", "%YAML:1.0", "---");
% write distortion
dist(:,1) = [left_stereo.CameraParameters1.RadialDistortion left_stereo.CameraParameters1.TangentialDistortion];
dist(:,2) = [left_stereo.CameraParameters2.RadialDistortion left_stereo.CameraParameters2.TangentialDistortion];
dist(:,3) = [right_stereo.CameraParameters1.RadialDistortion right_stereo.CameraParameters1.TangentialDistortion];
dist(:,4) = [right_stereo.CameraParameters2.RadialDistortion right_stereo.CameraParameters2.TangentialDistortion];
for c=1:4
    m = dist(:,c);
    fprintf(fileID, "%s%d%s\n   %s\n   %s\n   %s\n   %s", "Distortion", c-1,": !!opencv-matrix", "rows: 4", "cols: 1", "dt: f", "data: [");
    for i=1:3
        fprintf(fileID, "%.9f, ", m(i));
    end
    fprintf(fileID, "%.9f]\n", m(4));
end 


% write intrinsic matrixes
Intri(:,:,1) = left_stereo.CameraParameters1.IntrinsicMatrix;
Intri(:,:,2) = left_stereo.CameraParameters2.IntrinsicMatrix;
Intri(:,:,3) = right_stereo.CameraParameters1.IntrinsicMatrix;
Intri(:,:,4) = right_stereo.CameraParameters2.IntrinsicMatrix;
for c=1:4
    fprintf(fileID, "%s%d%s\n   %s\n   %s\n   %s\n   %s", "cameraMatrix", c-1,": !!opencv-matrix", "rows: 3", "cols: 3", "dt: f", "data: [");
    m = Intri(:,:,c).';
    for row=1:3
        for col=1:3
            fprintf(fileID, "%.9f", m(row,col));
            if row<3 || col<3
                fprintf(fileID, ", "); % cannot add "," after last element
            end
        end
    end
    fprintf(fileID, "]\n");
end
% translation of camera2 in left and right camera pair
Translation(:,:,1) = left_stereo.TranslationOfCamera2;
Translation(:,:,2) = right_stereo.TranslationOfCamera2;
for c=1:2
    fprintf(fileID, "%s%d%s\n   %s\n   %s\n   %s\n   %s", "Translation", c-1,": !!opencv-matrix", "rows: 3", "cols: 1", "dt: f", "data: [");
    m = Translation(:,:,c);
    for col=1:3
        fprintf(fileID, "%.9f", m(col));
        if col<3
            fprintf(fileID, ", "); % cannot add "," after last element
        end
    end
    fprintf(fileID, "]\n");
end
% rotation of camera2 in left and right camera pair
Rotation(:,:,1) = left_stereo.RotationOfCamera2;
Rotation(:,:,2) = right_stereo.RotationOfCamera2;
for c=1:2
    fprintf(fileID, "%s%d%s\n   %s\n   %s\n   %s\n   %s", "Rotation", c-1,": !!opencv-matrix", "rows: 3", "cols: 3", "dt: f", "data: [");
    m = Rotation(:,:,c);
    for row=1:3
        for col=1:3
            fprintf(fileID, "%.9f", m(row,col));
            if row<3 || col<3
                fprintf(fileID, ", "); % cannot add "," after last element
            end
        end
    end
    fprintf(fileID, "]\n");
end
fclose(fileID);