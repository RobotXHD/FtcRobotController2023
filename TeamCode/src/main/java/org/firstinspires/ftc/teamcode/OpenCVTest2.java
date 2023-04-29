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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCVTest2 extends LinearOpMode{
    private OpenCvWebcam webcam;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    public boolean prinde = false, stop = false;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    private double width, height;
    public double lastTime,var=1;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private PipelineAlbastru pipelineAlbastru = new PipelineAlbastru();
    private PipelineRosu pipelineRosu = new PipelineRosu();
    private  PipelineStalp pipelineStalp = new PipelineStalp();
    String varrez = "Mijloc";
    public void telemetrie(){
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("Rectangle Width:", width);
        telemetry.addData("Rectangle Height:", height);
        telemetry.addData("Rectangle H/W:", height / width);

        if (height / width > 1.2 && height / width < 2.4 && !prinde) {
            telemetry.addData("Rect", "1");
            varrez = "Stanga";
        } else if (height / width > 2.4 && !prinde) {
            telemetry.addData("Rect", "2");
            varrez = "Mijloc";
        } else {
            telemetry.addData("Rect", "3");
            varrez = "Dreapta";
        }
        telemetry.addData("caz", varrez);
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.setPipeline(pipelineRosu);
            webcam.setMillisecondsPermissionTimeout(2500);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
            telemetry.addLine("Waiting for start");
            telemetry.update();
            FtcDashboard.getInstance().startCameraStream(webcam, 60);
        while (!isStopRequested() && !isStarted()) {
            if(gamepad1.left_bumper) {
                var=1;
            }
            else if(gamepad1.right_bumper){
                var=3;
            }
            else{
                var=2;
            }
            try {
                if(var==1){
                    webcam.setPipeline(pipelineAlbastru);
                    width = pipelineAlbastru.getRect().width;
                    height = pipelineAlbastru.getRect().height;
                }
                else if(var==2){
                    webcam.setPipeline(pipelineRosu);
                    width = pipelineRosu.getRect().width;
                    height = pipelineRosu.getRect().height;
                }
                else{
                    webcam.setPipeline(pipelineStalp);
                    width = pipelineStalp.getRect().width;
                    height = pipelineStalp.getRect().height;
                }
                telemetrie();
                prinde = false;
            }
            catch (Exception E) {
                telemetry.addData("Webcam Error:", "Please Restart");
            }
            telemetry.addData("var:", var);
        }
        Autonom.start();
        while(!isStopRequested()){
            telemetrie();
        }
    }
    private final Thread Autonom = new Thread(new Runnable() {
        @Override
        public void run() {

        }
    });
}
