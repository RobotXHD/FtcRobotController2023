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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class PowerPlay_MECHTOWER extends OpMode {
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx ecstensor, turela;
    private DcMotorEx alecsticulator1,alecsticulator2;
    private Servo claw;
    private Servo supramax;
    double sm = 1, ms = 1.5,ms2 = 1;
    private BNO055IMU imu;
    double y, x, rx, rx2;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double colagen=0, lastTime;
    private boolean alast,lblast = false,rblast = false, slay=false,ylast;
    boolean v = true,ok1,ok2=false,ok3,ok4,ok5,ok6,ok7;
    private boolean stop=false,setSetpoint=false,setsetSetpoint=false;
    int okGrip = 1, okClaw = 1;
    public int i;
    public int cn=0;
    private double correction=0;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25, lbcn=0,rbcn=0,acn=0,ycn=0;
    int loaderState = -1;
    private int apoz = 0;
    private long spasmCurrentTime = 0;
    private long pidTime = 0;
    public double difference,medie;
    public double medii[] = new double[10];
    public boolean rotating = false,inAutomatizare = false;
    public double realAngle, targetAngle;
    private double forward, right, clockwise;
    double pidResult;
    PidControllerAdevarat pid = new PidControllerAdevarat(0.0,0.0,0.0);
    boolean setSetPoint=false;
    DigitalChannel touchL,touchR;
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
        alecsticulator1.setTargetPosition(poz1);
        alecsticulator2.setTargetPosition(-poz1);
        alecsticulator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        alecsticulator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        alecsticulator1.setPower(pow1);
        alecsticulator2.setPower(pow1);
        while (alecsticulator1.isBusy()||alecsticulator2.isBusy());
        alecsticulator1.setPower(0);
        alecsticulator2.setPower(0);
        alecsticulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alecsticulator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorBL = hardwareMap.get(DcMotorEx.class, "motorss"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorsd"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorfs"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorfd"); // Motor Front-Right
        turela      = hardwareMap.get(DcMotorEx.class, "motorTower");
        alecsticulator1 = hardwareMap.get(DcMotorEx.class,"motorArm1");
        alecsticulator2 = hardwareMap.get(DcMotorEx.class, "motorArm2");
        ecstensor = hardwareMap.get(DcMotorEx.class, "motorExtension");
        claw      = hardwareMap.servo.get("servoRelease");
        supramax = hardwareMap.servo.get("servoRotate");

        touchL = hardwareMap.get(DigitalChannel.class, "touchL"); // limitator de rotație stânga
        touchR = hardwareMap.get(DigitalChannel.class, "touchR"); // limitator de rotație dreapta

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

        turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        alecsticulator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        alecsticulator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ecstensor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        turela.setMode(DcMotor.RunMode.RESET_ENCODERS);
        alecsticulator1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        alecsticulator2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        ecstensor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //alecsticulator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //alecsticulator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ecstensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alecsticulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alecsticulator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ecstensor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                if(gamepad2.left_bumper) {
                    ok1 = false;
                    ok1 = true;
                }
                y  = gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
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
                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

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
                lblast = gamepad1.left_bumper;
                rblast = gamepad1.right_bumper;
                if(gamepad1.a){
                    turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    alecsticulator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    alecsticulator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ecstensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if(sm==3){
                    POWER(pmotorFR/1.5, pmotorFL/1.5, pmotorBR/1.5, pmotorBL/1.5);
                }
                else if(sm==5){
                    POWER(pmotorFR / 3, pmotorFL / 3, pmotorBR / 3, pmotorBL / 3);
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
                //if(ecstensor.getCurrentPosition() < 300) {
                ecstensor.setPower(gamepad2.left_stick_y/ms2);
                if (gamepad2.x != alast) {
                    acn++;
                }
                if(acn>=4){
                    acn = 0;
                }
                if(acn==2) {
                    ms = 3;
                }
                else if(acn==0){
                    ms = 1.5;
                }
                alast = gamepad2.x;
                if (gamepad2.y != ylast) {
                    ycn++;
                }
                if(ycn>=4){
                    ycn = 0;
                }
                if(ycn==2) {
                    ms2 = 3;
                }
                else if(ycn==0){
                    ms2 = 1.7;
                }
                if(gamepad2.dpad_left){
                    target(1500,0.4,turela);
                }
                ylast = gamepad2.y;
                alecsticulator1.setPower(gamepad2.right_stick_y);
                alecsticulator2.setPower(-gamepad2.right_stick_y);
                if(touchL.getState() && touchR.getState()) {
                    turela.setPower((gamepad2.left_trigger/ms - gamepad2.right_trigger/ms)/1.4);
                }
                else if(!touchR.getState() && !inAutomatizare) {
                    turela.setPower(-0.06);
                    inAutomatizare = true;
                }
                else if(!touchL.getState() && !inAutomatizare){
                    turela.setPower(0.06);
                    inAutomatizare = true;
                }
                inAutomatizare = false;
                if(gamepad2.left_bumper && colagen <= 1){
                    colagen+=0.005;
                }
                if(gamepad2.right_bumper && colagen >= 0){
                    colagen-=0.005;
                }
                supramax.setPosition(1-colagen);
                if(gamepad2.a /*&& colagen >= 0.01*/) {
                    //colagen -= 0.005;
                    claw.setPosition(0.2);
                }
                if(gamepad2.b /*&& colagen <= 0.99*/) {
                    //colagen+=0.005;
                    claw.setPosition(0.7);
                }

                //supramax.setPosition(1-colagen);

                if(gamepad2.dpad_up)
                {
                    inAutomatizare = true;
                    target(-732, 0.7,alecsticulator1);
                    target(1200,0.7,turela);
                    target(-110,0.7,ecstensor);
                    inAutomatizare = false;
                }


                if(gamepad2.dpad_down)
                {
                    inAutomatizare = true;
                    target(0,0.7,ecstensor);
                    while(touchL.getState()){
                        turela.setPower(-0.7);
                    }
                    target(0, 0.7,alecsticulator1);
                    turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turela.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    inAutomatizare = false;
                }

                /*if(gamepad2.dpad_up){
                    alecsticulator1.setTargetPosition(62);
                    alecsticulator2.setTargetPosition(0);
                    alecsticulator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    alecsticulator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    alecsticulator1.setPower(0.4);
                    alecsticulator2.setPower(-0.2);


                }*/

                boolean isOver=true;
               /* if(gamepad2.y && isOver==true){
                    isOver = false;
                    target(-10,1,ecstensor);
                    target(-1060,1,alecsticulator1);
                    target(-250,1,ecstensor);
                }*/
                isOver = true;
            }
        }
    });
    private final Thread pdi = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.setPID(constants.kp, constants.ki, constants.kd);

            while (!stop) {
                if (!turela.isBusy()) {
                    turela.setPower(gamepad2.left_trigger / 2 - gamepad2.right_trigger / 2);
                    setSetpoint = true;
                } else {
                    if (setSetpoint) {
                        pid.setSetpoint(turela.getCurrentPosition());
                        setSetpoint = false;
                    }
                    pidResult = pid.performPID(turela.getCurrentPosition());
                    turela.setPower(pidResult);
                }
            }
        }
    });
    public void loop() {
        telemetry.addData("motorBL:", motorBL.getCurrentPosition());
        telemetry.addData("motorBR:", motorBR.getCurrentPosition());
        telemetry.addData("motorFL:", motorFL.getCurrentPosition());
        telemetry.addData("motorFR:", motorFR.getCurrentPosition());
        telemetry.addData("ecstensor:", ecstensor.getCurrentPosition());
        telemetry.addData("alecsticulator1:", alecsticulator1.getCurrentPosition());
        telemetry.addData("alecsticulator2:", alecsticulator2.getCurrentPosition());
        telemetry.addData("alecsticulator1:", alecsticulator1.getPower());
        telemetry.addData("alecsticulator1:", alecsticulator2.getPower());
        telemetry.addData("supramax:", supramax.getPosition());
        telemetry.addData("colagen:", colagen);
        telemetry.addData("claw:", claw.getPosition());
        telemetry.addData("pozitie turela", turela.getCurrentPosition());
        telemetry.addData("sm:",sm);
        telemetry.addData("ms:",ms);
        telemetry.addData("rbcn:",rbcn);
        telemetry.addData("lbcn:",lbcn);
        telemetry.addData("STARE SENZOR stg",touchL.getState());
        telemetry.addData("STARE SENZOR stg",touchR.getState());
        telemetry.addData("getSetpoint",pid.getSetpoint());
        telemetry.addData("PError",pid.getError()*pid.getP());
        telemetry.addData("DError",pid.getDError()*pid.getD());
        telemetry.addData("IError",pid.getISum()*pid.getI());
        telemetry.addData("Error",pid.getError());
        telemetry.update();
    }
}
