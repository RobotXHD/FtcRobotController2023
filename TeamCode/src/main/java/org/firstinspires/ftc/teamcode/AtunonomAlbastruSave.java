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
public class AtunonomAlbastruSave extends LinearOpMode{
    boolean ipd = true;
    private OpenCvWebcam webcam;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    private double width, height;
    public double lastTime;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private DcMotorEx motorFR, motorFL, motorBR, motorBL;
    private DcMotorEx ecstensor,turela;
    private DcMotorEx alecsticulator1,alecsticulator2;
    private Servo crow;
    private Servo supramax;
    DigitalChannel touchL,touchR;
    String varrez = "Mijloc";
    PidControllerAdevarat pid = new PidControllerAdevarat(0.0,0.0,0.0);
    public int poz2=0;
    @Override
    public void runOpMode() throws InterruptedException {
        pid.enable();
        pid.setPID(constants.kp, constants.ki, constants.kd);
        motorBL = hardwareMap.get(DcMotorEx.class, "motorss"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorsd"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorfs"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorfd"); // Motor Front-Right

        turela      = hardwareMap.get(DcMotorEx.class, "motorTower");
        alecsticulator1 = hardwareMap.get(DcMotorEx.class,"motorArm1");
        alecsticulator2 = hardwareMap.get(DcMotorEx.class, "motorArm2");
        ecstensor = hardwareMap.get(DcMotorEx.class, "motorExtension");

        crow      = hardwareMap.servo.get("servoRelease");
        supramax = hardwareMap.servo.get("servoRotate");

        touchL = hardwareMap.get(DigitalChannel.class, "touchL");
        touchR = hardwareMap.get(DigitalChannel.class, "touchR");

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        alecsticulator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        alecsticulator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ecstensor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        alecsticulator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        alecsticulator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ecstensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alecsticulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alecsticulator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ecstensor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        while (!isStarted() && !isStopRequested()) {
            try {
                width = pipeline.getRect().width;
                height = pipeline.getRect().height;
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
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
            catch (Exception E){
                height = 1;
                width = 1000;
                varrez = "Dreapta";
                telemetry.addData("Webcam Error:", "Please Restart");
            }
            telemetry.addData("caz",varrez);
        }
        if(!isStopRequested()) {
            pdi.start();
            Autonom.start();
        }
        while(!isStopRequested()){
            telemetry.addData("turela:",turela.getCurrentPosition());
            telemetry.update();
        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            if(false) {
                supramax.setPosition(0.5);
                kdf(600);
                crow.setPosition(0.2);
                kdf(400);
                supramax.setPosition(0.8);
                kdf(200);
                Translatare(30, 0, 0.5);
                kdf(200);
                Translatare(0, -200, 0.5);
                kdf(200);
                target(-975, 0.75, alecsticulator1);
                kdf(200);
                Rotire(-60, 0.2);
                kdf(200);
                target(-380, 0.75, ecstensor);
                kdf(200);
                supramax.setPosition(0);
                kdf(900);
                crow.setPosition(0.7);
                kdf(100);
                target(-660, 0.3, turela);
                kdf(200);
                supramax.setPosition(0.5);
                kdf(900);
                target(-520, 0.5, alecsticulator1);
                kdf(700);
                target(-400, 0.5, ecstensor);
                kdf(200);
                crow.setPosition(0.2);
                kdf(100);
                supramax.setPosition(0.8);
                kdf(100);
                target(-570, 0.5, alecsticulator1);
                kdf(100);
                target(-380, 0.5, ecstensor);
                kdf(100);
                target2(-975, 0.5);
                kdf(200);
                target(0, 0.5, ecstensor);
                kdf(200);
                target(10, 0.3, turela);
                kdf(100);
                target(-365, 0.5, ecstensor);
                kdf(100);
                supramax.setPosition(0);
                kdf(100);
                crow.setPosition(0.7);
            }
            /*
            kdf(100);
            target(-610,0.5,turela);
            kdf(200);
            supramax.setPosition(0.5);
            kdf(900);
            target(-540,0.5,alecsticulator1);
            kdf(100);
            target(-430,0.5,ecstensor);
            kdf(200);
            crow.setPosition(0.2);
            kdf(100);
            target(-1000,0.5,alecsticulator1);
            kdf(200);
            target(0,0.5,ecstensor);
            kdf(200);
            target(0,0.5,turela);
            kdf(100);
            target(-365,0.5,ecstensor);
            kdf(100);
            crow.setPosition(0.7);*/
            /*
            target(-840,0.5,alecsticulator1);
            kdf(600);
            target(275,0.5,turela);
            kdf(1000);
            supramax.setPosition(0.1);
            kdf(200);
            Translatare(0,-10,0.4);
            kdf(200);
            target(-450,0.5,ecstensor);
            kdf(400);
            crow.setPosition(0.7);
            kdf(500);
            target(-20,0.5,ecstensor);
            kdf(600);
            target(-1400,0.5,alecsticulator1);
            Translatare(135,0,0.4);
            kdf(300);
            Translatare(0,-243,0.4);
            kdf(200);
            target(-1250,0.5,turela);
            kdf(200);
            Translatare(-195,0,0.4);*/
            if(varrez == "Mijloc"&&!isStopRequested()) {
                supramax.setPosition(0.5);
                kdf(600);
                crow.setPosition(0.2);
                kdf(400);
                supramax.setPosition(0.8);
                kdf(200);
                Translatare(30,0,0.5);
                kdf(200);
                Translatare(0,-200,0.5);
                kdf(200);
                target(-950,0.8,alecsticulator1);
                kdf(200);
                Rotire(-85,0.2);
                kdf(200);
                target(-300,0.8,ecstensor);
                kdf(200);
                supramax.setPosition(0.2);
                kdf(900);
                crow.setPosition(0.7);
                kdf(100);
                target(0,0.5,ecstensor);
                kdf(100);
                target(-660,0.3,turela);
                kdf(200);
                supramax.setPosition(0.5);
                kdf(900);
                target(-520,0.8,alecsticulator1);
                kdf(700);
                target(-400,0.8,ecstensor);
                kdf(200);
                crow.setPosition(0.2);
                kdf(100);
                supramax.setPosition(0.8);
                kdf(100);
                target(-570,0.8,alecsticulator1);
                kdf(100);
                target(-380,0.8,ecstensor);
                kdf(100);
                target2(-975,0.8);
                kdf(200);
                target(0,0.8,ecstensor);
                kdf(200);
                target(10,0.3,turela);
                kdf(100);
                target(-365,0.8,ecstensor);
                kdf(100);
                supramax.setPosition(0.2);
                kdf(100);
                crow.setPosition(0.7);
            }
            if(varrez == "Stanga"&&!isStopRequested()) {
                Translatare(-260,0,0.4);
            }
        }
    });
    private final Thread pdi = new Thread(new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                double pidResult;
                if(ipd == true) {
                    pid.setSetpoint(poz2);
                    pidResult = pid.performPID(poz2);
                    turela.setPower(pidResult);
                }
            }
        }
    });
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }
    public void target(int poz, double pow, DcMotorEx motor){
        ipd = false;
        poz2 = poz;
        motor.setTargetPosition(poz);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        while (motor.isBusy());
        motor.setPower(0);
        ipd = true;
        /*pid.setSetpoint(motor.getCurrentPosition());
        pid_result = pid.performPID(motor.getCurrentPosition());
        motor.setPower(pid_result);*/
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void target2(int poz, double pow){
        alecsticulator1.setTargetPosition(poz);
        alecsticulator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        alecsticulator1.setPower(pow);
        alecsticulator2.setPower(pow);
        while (alecsticulator1.isBusy());
        alecsticulator2.setPower(0);
        alecsticulator1.setPower(0);
        alecsticulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void targeturela(int poz, double pow){
        double pid_result;
        turela.setTargetPosition(poz);
        turela.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turela.setPower(pow);
        while (turela.isBusy());
        turela.setPower(0);
        turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void kdf(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime + t > System.currentTimeMillis()){

        }
    }
}
