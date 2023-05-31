package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class VarPipelineAlbastru {
    public enum DetectionTypes {
        DAY,
        NIGHT
    }
    public static double AlDay_Hhigh = 130, AlDay_Shigh = 255, AlDay_Vhigh = 255, AlDay_Hlow = 90, AlDay_Slow = 0, AlDay_Vlow = 0;
    public static double AlNight_Hhigh = 90, AlNight_Shigh = 255, AlNight_Vhigh = 200, AlNight_Hlow = 30, AlNight_Slow = 0, AlNight_Vlow = 100;
    public static DetectionTypes AlCV_detectionType = DetectionTypes.DAY;
}
