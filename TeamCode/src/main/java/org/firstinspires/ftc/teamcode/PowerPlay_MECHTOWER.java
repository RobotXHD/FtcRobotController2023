package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class PowerPlay_MECHTOWER extends OpMode {
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx extensor, turela;
    private DcMotorEx articulator1,articulator2;
    private Servo claw;
    private Servo clawArticulation;
    private Servo clawRotation;
    double sm = 1, ms = 1.5,ms2 = 1;
    private BNO055IMU imu;
    double y, x, rx, rx2,brat;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double colagen=0, lastTime, rotatiometru=0;
    private boolean alast,lblast = false,rblast = false, slay=false,ylast;
    boolean v = true,ok1,ok2=false,ok3,ok4,ok5,ok6,ok7;
    private boolean stop=false,setSetpoint=false,setsetSetpoint=false;
    int okGrip = 1, okClaw = 1;
    public int i;
    public int cn=0;
    private double correction=0, lastPos=0;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25, lbcn=0,rbcn=0,acn=0,ycn=0;
    int loaderState = -1;
    private int apoz = 0;
    private long spasmCurrentTime = 0;
    private long pidTime = 0;
    public double difference,medie,poz2=0;
    public double medii[] = new double[10];
    public boolean rotating = false,inAutomatizare = true;
    public double realAngle, targetAngle;
    private double forward, right, clockwise;
    double pidResult;
    private HardwareRobotTurela f = new HardwareRobotTurela();
    PidControllerAdevarat pid = new PidControllerAdevarat(0.0,0.0,0.0);
    boolean setSetPoint=false;
    TouchSensor touchL,touchR;
    void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
    public void target(int poz, double pow, DcMotorEx motor){
        motor.setTargetPosition(poz);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(pow);
        while (motor.isBusy());
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void mamplicitisit(int poz1, double pow1){
        articulator1.setTargetPosition(poz1);
        articulator2.setTargetPosition(-poz1);
        articulator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        articulator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        articulator1.setPower(pow1);
        articulator2.setPower(pow1);
        while (articulator1.isBusy()||articulator2.isBusy());
        articulator1.setPower(0);
        articulator2.setPower(0);
        articulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        articulator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorBL = hardwareMap.get(DcMotorEx.class, "motorss"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorsd"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorfs"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorfd"); // Motor Front-Right
        //turela      = hardwareMap.get(DcMotorEx.class, "motorTower");
        articulator1 = hardwareMap.get(DcMotorEx.class,"motorArm1");
        articulator2 = hardwareMap.get(DcMotorEx.class, "motorArm2");
        extensor = hardwareMap.get(DcMotorEx.class, "motorExtension");
        claw      = hardwareMap.servo.get("servoRelease");
        clawArticulation = hardwareMap.servo.get("servoArticulate");
        clawRotation = hardwareMap.servo.get("servoRotate");

        touchL = hardwareMap.get(TouchSensor.class, "touchL"); // limitator de rotație stânga
        touchR = hardwareMap.get(TouchSensor.class, "touchR"); // limitator de rotație dreapta

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //(turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        articulator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        articulator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        articulator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        articulator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        articulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        articulator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();


    }
    public void start(){
        Chassis.start();
        Systems.start();
        //pdi.start();
    }
    public void stop(){stop = true;}
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop) {
                if(gamepad1.dpad_up){
                    inAutomatizare = false;
                }
                articulator1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(constants.kp,constants.ki,constants.kd,constants.kf));
                //turela.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(constants.kp,constants.ki,constants.kd,constants.kf));
                y  = gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
                rx2 = gamepad2.right_stick_x / 7;
                /*
                pid.setPID(constants.pGyro,constants.iGyro,constants.dGyro);
                if(clockwise != 0.0){
                    correction = 0.0;
                    rotating = true;
                }
                else{
                    if((forward != 0.0 || right != 0.0) && Math.abs(medie) < 0.5) {
                        if (rotating) {
                            targetAngle = realAngle;
                            rotating = false;
                            pid.setSetpoint(targetAngle);
                        }
                        correction = pid.performPID(realAngle);
                    }
                    else{
                        correction = 0.0;
                    }
                }*/
                pmotorFL = -y + x + rx + rx2;
                pmotorBL = -y - x + rx + rx2;
                pmotorBR = -y + x - rx - rx2;
                pmotorFR = -y - x - rx - rx2;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                //SLOW-MOTION
                if (gamepad1.left_bumper != lblast) {
                    lbcn++;
                }
                if(lbcn>=4){
                    lbcn = 0;
                }
                if(lbcn==2 && sm!=5) {
                    sm = 3;
                }
                if (gamepad1.right_bumper != rblast) {
                    rbcn++;
                }
                if(rbcn>=4){
                    rbcn = 0;
                }
                if(rbcn==2) {
                    sm = 5;
                }
                else if(rbcn==0 && lbcn == 0){
                    sm=1;
                }
                rblast = gamepad1.right_bumper;
                lblast = gamepad1.left_bumper;
                if(gamepad1.x){
                    //turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    articulator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    articulator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if(sm==3){
                    POWER(pmotorFR * 1.2, pmotorFL * 1.2, pmotorBR * 1.2, pmotorBL * 1.2);
                }
                else if(sm==5){
                    POWER(pmotorFR / 2, pmotorFL / 2, pmotorBR / 2, pmotorBL / 2);
                }
                else{
                    POWER(pmotorFR * 2, pmotorFL * 2, pmotorBR * 2, pmotorBL * 2);
                }
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                //if(extensor.getCurrentPosition() < 300) {
                extensor.setPower(gamepad2.left_stick_y/1.75/ms2);
//                if(gamepad2.dpad_left){
//                    target(1500,0.4,turela);
//                }

                ylast = gamepad2.y;
                    articulator1.setPower(-gamepad2.right_stick_y / 2);
                    articulator2.setPower(gamepad2.right_stick_y / 2);
                /*if(!touchL.isPressed() && !touchR.isPressed()) {
                    turela.setPower((gamepad2.left_trigger/ms - gamepad2.right_trigger/ms)0/1.4);
                }
                else if(touchR.isPressed() && !inAutomatizare) {
                    turela.setPower(-0.06);
                    inAutomatizare = true;
                }
                else if(touchL.isPressed() && !inAutomatizare){
                    turela.setPower(0.06);
                    inAutomatizare = true;
                }*/
                inAutomatizare = false;
                if(gamepad2.left_bumper && colagen <= 1){
                    colagen+=0.001;
                }
                if(gamepad2.right_bumper && colagen >= 0){
                    colagen-=0.001;
                }
                clawArticulation.setPosition(1-colagen);
                if(gamepad2.a /*&& colagen >= 0.01*/) {
                    //colagen -= 0.005;
                    claw.setPosition(0.3);
                }
                if(gamepad2.b /*&& colagen <= 0.99*/) {
                    //colagen+=0.005;
                    claw.setPosition(1);
               }
                /*if (gamepad2.right_trigger > 0 && poz2 <= 0.98){
                    poz2+= 0.002;
                }
                if(gamepad2.left_trigger > 0 && poz2 >= 0.02 ){
                    poz2-= 0.002;
                }*/
                if(gamepad2.x)
                    poz2=0.0;
                if(gamepad2.y)
                    poz2=0.7;
                clawRotation.setPosition(poz2);

//0.02 0.66
                if(gamepad2.dpad_down){
                    f.Rotire(75,0.5);
                    f.kdf(200);
                    f.target2(-220,0.5);
                    f.kdf(200);
                    claw.setPosition(1);
                    f.kdf(200);
                    clawArticulation.setPosition(0.1);
                }
                /*if(gamepad2.dpad_up){
                    articulator1.setTargetPosition(62);
                    articulator2.setTargetPosition(0);
                    articulator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    articulator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    articulator1.setPower(0.4);
                    articulator2.setPower(-0.2);


                }*/

                boolean isOver=true;
               /* if(gamepad2.y && isOver==true){
                    isOver = false;
                    target(-10,1,extensor);
                    target(-1060,1,articulator1);
                    target(-250,1,extensor);
                }*/
                isOver = true;
            }
        }
    });
//    private final Thread pdi = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            pid.enable();
//            while (!stop) {
//                pid.setPID(constants.kp, constants.ki, constants.kd);
//                if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
//                    turela.setPower(gamepad2.left_trigger / 2 - gamepad2.right_trigger / 2);
//                    setSetpoint = true;
//                }
//                else {
//                    if (setSetpoint) {
//                        pid.setSetpoint(turela.getCurrentPosition());
//                        setSetpoint = false;
//                    }
//                    pidResult = pid.performPID(turela.getCurrentPosition());
//                    turela.setPower(pidResult);
//                }
//            }
//        }
//    });
    public void loop() {
        telemetry.addData("motorBL:", motorBL.getCurrentPosition());
        telemetry.addData("motorBR:", motorBR.getCurrentPosition());
        telemetry.addData("motorFL:", motorFL.getCurrentPosition());
        telemetry.addData("motorFR:", motorFR.getCurrentPosition());
        telemetry.addData("lastPos:", lastPos);
        telemetry.addData("extensor:", extensor.getCurrentPosition());
        telemetry.addData("articulator1:", articulator1.getCurrentPosition());
        telemetry.addData("articulator2:", articulator2.getCurrentPosition());
        telemetry.addData("clawArticulation:", clawArticulation.getPosition());
        telemetry.addData("clawRotation:", clawRotation.getPosition());
        telemetry.addData("colagen:", colagen);
        telemetry.addData("claw:", claw.getPosition());
        telemetry.addData("sm:",sm);
        telemetry.addData("rotatie",clawRotation.getPosition());
        telemetry.addData("ms:",ms);
        telemetry.addData("rbcn:",rbcn);
        telemetry.addData("lbcn:",lbcn);
        telemetry.addData("STARE SENZOR stg",touchL.isPressed());
        telemetry.addData("STARE SENZOR stg",touchR.isPressed());
        telemetry.addData("getSetpoint",pid.getSetpoint());
        telemetry.addData("PError",pid.getError()*pid.getP());
        telemetry.addData("DError",pid.getDError()*pid.getD());
        telemetry.addData("IError",pid.getISum()*pid.getI());
        telemetry.addData("Error",pid.getError());
        //telemetry.addData("putere turela", turela.getPower());
        telemetry.update();
    }
}
