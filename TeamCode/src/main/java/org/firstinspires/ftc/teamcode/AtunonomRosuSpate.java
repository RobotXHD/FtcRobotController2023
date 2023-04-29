package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

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
public class AtunonomRosuSpate extends LinearOpMode {
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
        while (!isStarted() && !isStopRequested()) {
            try {
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
                }
                else if (height / width > 2.4) {
                    telemetry.addData("Rect", "2");
                    varrez = "Mijloc";
                } else {
                    telemetry.addData("Rect", "3");
                    varrez = "Dreapta";
                }
                telemetry.addData("caz", varrez);
                telemetry.update();
            } catch (Exception E) {
                height = 1;
                width = 1000;
                varrez = "Dreapta";
                telemetry.addData("Webcam Error:", "Please Restart");
            }
            telemetry.addData("caz", varrez);
        }
        if (!isStopRequested()) {
            Autonom.start();
        }
        while (opModeIsActive()) {
            //telemetry.addData("turela:", f.turela.getCurrentPosition());
            //telemetry.addData("ecstensor:", f.ecstensor.getCurrentPosition());
            telemetry.addData("BRisBusy:", f.motorBR.isBusy());
            telemetry.addData("BLisBusy:", f.motorBL.isBusy());
            telemetry.addData("FRisBusy:", f.motorFR.isBusy());
            telemetry.addData("FLisBusy:", f.motorFL.isBusy());
            telemetry.update();
        }
    }

    public Thread Autonom = new Thread(() -> {
        /*f.supramax.setPosition(0.5);
        f.kdf(600);
        f.crow.setPosition(0.2);
        f.kdf(400);
        f.supramax.setPosition(0.8);
        f.kdf(200);
        f.target2(-750, 0.5);
        f.kdf(100);`
        f.targetime(-180, 0.1, f.turela,5000);
        f.kdf(200);
        f.targetime(-555, 0.5, f.ecstensor,1200);
        f.kdf(200);
        f.supramax.setPosition(0.4);
        f.kdf(900);
        f.crow.setPosition(0.7);
        f.kdf(100);
        f.targetime(0,0.5,f.ecstensor,1200);
        f.kdf(200);
        f.supramax.setPosition(1);
        f.targetime(-1400,0.5, f.alecsticulator1,1200);
        f.kdf(100);*/
        f.TranslatareTimp(130,0,0.5,200);
        f.kdf(500);
        f.Translatare(0, 130, 0.5);
        if(varrez == "Stanga"){
            f.TranslatareTimp(-130,0,0.5,2000);
        }
        else if(varrez == "Dreapta"){
            f.TranslatareTimp(130,0,0.5,2000);
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
