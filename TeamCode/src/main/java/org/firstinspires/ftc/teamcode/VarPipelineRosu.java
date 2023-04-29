package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class VarPipelineRosu {
    public enum DetectionTypes {
        DAY,
        NIGHT
    }
    public static double RoDay_Hhigh = 180, RoDay_Shigh = 255, RoDay_Vhigh = 255, RoDay_Hlow = 120, RoDay_Slow = 50, RoDay_Vlow = 0;
    public static double RoNight_Hhigh = 90, RoNight_Shigh = 255, RoNight_Vhigh = 200, RoNight_Hlow = 30, RoNight_Slow = 0, RoNight_Vlow = 100;
    public static int CV_kernel_pult_size = 5, Webcam_w = 640, Webcam_h = 480, CV_rect_x1 = 0, CV_rect_y1 = 0, CV_rect_x2 = 640, CV_rect_y2 = 480;
    public static DetectionTypes RoCV_detectionType = DetectionTypes.DAY;
}
