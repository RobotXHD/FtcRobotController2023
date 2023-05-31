package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;
import static org.firstinspires.ftc.teamcode.constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCVTest2 extends LinearOpMode{
    private OpenCvWebcam webcam;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double correction;
    PidControllerAdevarat pid = new PidControllerAdevarat(cp,ci,cd);
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    public boolean prinde = false, stop = false;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    private double width, height;
    public double lastTime,var=1;
    int cameraMonitorViewId;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private PipelineAlbastru pipelineAlbastru = new PipelineAlbastru();
    private PipelineRosu pipelineRosu = new PipelineRosu();
    private  PipelineStalp pipelineStalp = new PipelineStalp();
    String varrez = "Mijloc";
    public void telemetrie(){
        telemetry.addData("Rectangle Width:", width);
        telemetry.addData("Rectangle Height:", height);
        telemetry.addData("Rectangle H/W:", height / width);

        telemetry.addData("setpoint:",pid.getSetpoint());
        telemetry.addData("error",pid.getError());
        telemetry.addData("correction",correction);
        telemetry.update();
    }
    public void webcamInit(OpenCvPipeline pipeline){

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    @Override
    public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcamInit(pipelineAlbastru);
            webcam.setMillisecondsPermissionTimeout(2500);
            FtcDashboard.getInstance().startCameraStream(webcam, 60);
            pid.setSetpoint(Webcam_w / 2);
            pid.setPID(cp,ci,cd);
            pid.enable();
        while (opModeInInit()) {

        }
        while(opModeIsActive()){
            telemetry.addData("da","da");
            telemetry.update();
            webcamInit(pipelineRosu);
            try {
                correction = pipeline.getRect().x + pipeline.getRect().width / 2 - pid.getSetpoint();
                telemetrie();
            }
            catch (Exception E){
                telemetry.addData("Webcam Error:", E);
                telemetry.update();
            }
        }
    }
}
