package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class VarPipelineStalp {
    public enum DetectionTypes {
        DAY,
        NIGHT
    }
    public static double GaDay_Hhigh = 40, GaDay_Shigh = 255, GaDay_Vhigh = 255, GaDay_Hlow = 15, GaDay_Slow = 20, GaDay_Vlow = 0;
    public static double GaNight_Hhigh = 90, GaNight_Shigh = 255, GaNight_Vhigh = 200, GaNight_Hlow = 30, GaNight_Slow = 0, GaNight_Vlow = 100;
    public static double kp = 0.000002, ki = 0, kd = 0.0002;
    public static int CV_kernel_pult_size = 5, Webcam_w = 640, Webcam_h = 480, CV_rect_x1 = 0, CV_rect_y1 = 0, CV_rect_x2 = 640, CV_rect_y2 = 480;
    public static DetectionTypes GaCV_detectionType = DetectionTypes.DAY;
}
