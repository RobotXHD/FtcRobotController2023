package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;
import static org.firstinspires.ftc.teamcode.VarPipelineStalp.offsetWebcamG;
import static org.firstinspires.ftc.teamcode.constants.cd;
import static org.firstinspires.ftc.teamcode.constants.ci;
import static org.firstinspires.ftc.teamcode.constants.cp;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTest3 extends OpMode {
    private OpenCvWebcam webcam;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double correction;
    int target;
    PidControllerAdevarat pid = new PidControllerAdevarat(cp,ci,cd);
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    public boolean prinde = false, stop = false, ok = false;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    private double width, height,x,y;
    public double lastTime,var=1;
    int cameraMonitorViewId;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private PipelineAlbastru pipelineAlbastru = new PipelineAlbastru();
    private PipelineRosu pipelineRosu = new PipelineRosu();
    private  PipelineStalp pipelineStalp = new PipelineStalp();
    String varrez = "Mijloc";
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    public double motorPower = 0.0;
    void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setVelocity(df1);
        motorBL.setVelocity(ss1);
        motorFL.setVelocity(sf1);
        motorBR.setVelocity(ds1);
    }
    public void telemetrie(){
        telemetry.addData("Rectangle Width:", width);
        telemetry.addData("Rectangle Height:", height);
        telemetry.addData("Rectangle x:", x);
        telemetry.addData("Rectangle y:", y);
        telemetry.addData("ok",ok);
        telemetry.addData("motorPower:",motorPower);
        telemetry.addData("setpoint:",pid.getSetpoint());
        telemetry.addData("error",pid.getError());
        telemetry.addData("correction",correction);
        telemetry.addData("P:",pid.getP());
        telemetry.addData("I:",pid.getI());
        telemetry.addData("D:",pid.getD());
        telemetry.update();
    }
    public void stop(){stop = true;}
    @Override
    public void init() {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorss"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorsd"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorfs"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorfd"); // Motor Front-Right

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        pid.setSetpoint(Webcam_w / 2);
        pid.setPID(cp,ci,cd);
        pid.enable();
    }

    @Override
    public void loop() {
        pid.setPID(cp,ci,cd);
        webcam.setPipeline(pipelineRosu);
        try {
            width = pipelineRosu.getRect().width;
            x = pipelineRosu.getRect().x;
            /*correction = pid.performPID(x + width / 2 + offsetWebcamG);
            target = (int)(motorBL.getCurrentPosition() - pid.getError());
            if(pid.getError() > 2 || pid.getError() < -2){
                motorPower = correction / abs(pid.getError());
            }
            else{
                motorPower = 0;
            }
            POWER(motorPower,-motorPower,-motorPower,motorPower);*/
            correction = pid.performPID(x + width / 2);
            if(correction >= -2 && correction <= 2){
                correction = 0;
            }
            POWER(correction,-correction,-correction,correction);
            telemetrie();
        }
        catch (Exception E) {
            telemetry.addData("Webcam Error:", E);
            telemetry.update();
        }
    }
}
