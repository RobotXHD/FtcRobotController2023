package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Nu extends LinearOpMode{
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
    @Override
    public void runOpMode() throws InterruptedException {
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

//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(pipeline);
//
//        webcam.setMillisecondsPermissionTimeout(2500);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
        telemetry.addLine("Waiting for start");
        telemetry.update();
        //FtcDashboard.getInstance().startCameraStream(webcam, 60);

        while (!isStarted() && !isStopRequested()) {
//            try {
//                width = pipeline.getRect().width;
//                height = pipeline.getRect().height;
//                telemetry.addData("Frame Count", webcam.getFrameCount());
//                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
//                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
//                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
//                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
//                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
//                telemetry.addData("Rectangle Width:", width);
//                telemetry.addData("Rectangle Height:", height);
//                telemetry.addData("Rectangle H/W:", height / width);
//                if (height / width > 1.2 && height / width < 2.4) {
//                    telemetry.addData("Rect", "1");
//                    varrez = "Stanga";
//                } else if (height / width > 2.4) {
//                    telemetry.addData("Rect", "2");
//                    varrez = "Mijloc";
//                } else {
//                    telemetry.addData("Rect", "3");
//                    varrez = "Dreapta";
//                }
//                telemetry.addData("caz", varrez);
//                telemetry.update();
//            }
//            catch (Exception E){
//                height = 1000;
//                width = 1;
//                telemetry.addData("Webcam Error:", "Please Restart");
//            }
        }
        if(!isStopRequested()) {
            Autonom.start();
        }
        while(!isStopRequested()){

        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
//            crow.setPosition(0.2);
//            kdf(400);
//            target(-770,1,alecsticulator1);
//            kdf(600);
//            target(-520,0.5,ecstensor);
//            kdf(1000);
//            supramax.setPosition(0.2);
//            kdf(1000);
//            crow.setPosition(0.7);
//            kdf(500);
//            target(-20,0.5,ecstensor);
//            kdf(600);
            Translatare(0,-100,0.6);
            Rotire(30,0.4);

//            if(varrez=="Stanga"&&!isStopRequested()) {
//                Translatare(0,70,0.1);
//                kdf(200);
//                Translatare(-50,0,0.1);
                  Translatare(-120,0,0.5);
//            }
//            if(varrez == "Mijloc"&&!isStopRequested()) {
//                Translatare(0,70,0.1);
//            }
//
//            if(varrez == "Dreapta"&&!isStopRequested()) {
//                Translatare(0,70,0.1);
//                kdf(200);
//                Translatare(50,0,0.1);
                  Translatare(120,0,0.5);
//            }
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
        motor.setTargetPosition(poz);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        while (motor.isBusy());
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void targetime(int poz, double pow, DcMotorEx motor,int t){
        double lastTime2;
        lastTime2 = System.currentTimeMillis();
        while(lastTime2 + t > System.currentTimeMillis()) {
            motor.setTargetPosition(poz);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while (motor.isBusy());
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void kdf(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime + t > System.currentTimeMillis()){

        }
    }
}

