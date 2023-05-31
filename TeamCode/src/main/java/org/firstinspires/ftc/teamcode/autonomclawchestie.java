package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class autonomclawchestie extends LinearOpMode {
    boolean ipd = true;
    private double width, height;
    private String varrez = "Mijloc";

    private HardwareRobotTurela f = new HardwareRobotTurela();

    @Override
    public void runOpMode() throws InterruptedException {
        f.init(hardwareMap);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(f.webcam, 60);
        /*while (opModeInInit() && !isStopRequested()) {
            try {
                if(gamepad1.left_bumper){
                    CV_detectionType = Var.DetectionTypes.NIGHT;
                }
                if(gamepad1.right_bumper){
                    CV_detectionType = Var.DetectionTypes.DAY;
                }
                width = f.pipeline.getRect().width;
                height = f.pipeline.getRect().height;
                telemetry.addData("Frame Count", f.webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", f.webcam.getFps()));
                telemetry.addData("Total frame time ms", f.webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", f.webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", f.webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", f.webcam.getCurrentPipelineMaxFps());
                telemetry.addData("Rectangle Width:", width);
                telemetry.addData("Rectangle Height:", height);
                telemetry.addData("Rectangle H/W:", height / width);
                if (height / width > 1.2 && height / width < 2.4) {
                    telemetry.addData("Rect", "1");
                    varrez = "Stanga";
                } else if (height / width > 2.4) {
                    telemetry.addData("Rect", "2");
                    varrez = "Mijloc";
                } else {
                    telemetry.addData("Rect", "3");
                    varrez = "Dreapta";
                }
                telemetry.addData("caz", varrez);
                telemetry.update();
            }
            catch (Exception E) {
                height = 1;
                width = 1000;
                varrez = "Dreapta";
                telemetry.addData("Webcam Error:", "Please Restart");
            }
            telemetry.addData("caz", varrez);
        }*/
        waitForStart();
        if (!isStopRequested()) {
            Autonom.start();
        }
        while (!isStopRequested()) {
            //telemetry.addData("turela:", f.turela.getCurrentPosition());
            //telemetry.addData("ecstensor:", f.ecstensor.getCurrentPosition());
            //telemetry.addData("alecsticulator:", f.alecsticulator1.getCurrentPosition());
            telemetry.update();
        }
    }

    public Thread Autonom = new Thread(() -> {
        if(!isStopRequested()) {
            f.TranslatareTimp(65,0,0.5,1000);
            f.TranslatareTimp(0,65,0.5,1000);
            f.TranslatareTimp(-65,0,0.5,1000);
            f.TranslatareTimp(0,-65,0.5,1000);
        }
    });
    private final Thread pdi = new Thread(() -> {
        while (opModeIsActive()) {
            double pidResult;
            //f.turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(constants.kp,constants.ki,constants.kd,constants.kf));
            /*if(ipd == true) {
                pid.setSetpoint(poz2);
                pidResult = pid.performPID(poz2);
                turela.setPower(pidResult);
            }*/
        }
    });
}
