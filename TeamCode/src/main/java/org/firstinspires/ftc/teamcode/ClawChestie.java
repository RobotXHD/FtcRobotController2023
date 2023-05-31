package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ClawChestie extends OpMode{
public DcMotorEx MFS;
    public DcMotorEx MFD;
    public DcMotorEx MSS;
    public DcMotorEx MSD;
    public boolean stop=false;
    public double y,x,rx,pmotorFL,pmotorBL,pmotorBR ,pmotorFR,max,var=0;
    private DcMotorEx extensor;
    private DcMotorEx articulator1,articulator2;
    private Servo claw;
    private Servo clawArticulation;
    private Servo clawRotation;
    void POWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEER(double df1, double sf1, double ds1, double ss1){
        MFD.setPower(df1);
        MSS.setPower(ss1);
        MFS.setPower(sf1);
        MSD.setPower(ds1);
    }
    @Override

    public void init() {
        MSS = hardwareMap.get(DcMotorEx.class, "motorss"); // Motor Back-Left
        MSD= hardwareMap.get(DcMotorEx.class, "motorsd"); // Motor Back-Right
        MFS = hardwareMap.get(DcMotorEx.class, "motorfs"); // Motor Front-Left
        MFD = hardwareMap.get(DcMotorEx.class, "motorfd"); // Motor Front-Right

            articulator1 = hardwareMap.get(DcMotorEx.class,"motorArm1");
            articulator2 = hardwareMap.get(DcMotorEx.class, "motorArm2");
                extensor = hardwareMap.get(DcMotorEx.class, "motorExtension");
                     claw= hardwareMap.servo.get("servoRelease");
        clawArticulation = hardwareMap.servo.get("servoArticulate");
            clawRotation = hardwareMap.servo.get("servoRotate");

        articulator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        articulator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        articulator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        articulator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        articulator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        articulator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MSS.setDirection(DcMotorEx.Direction.REVERSE);
        MFS.setDirection(DcMotorEx.Direction.REVERSE);

        MSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MFS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void start(){
        Chassis.start();
        GARA.start();
    }

    public void stop(){stop = true;}
    private final Thread Chassis = new Thread(new Runnable() {

        @Override
        public void run() {
            while(stop==false){

                y  = gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                pmotorFL = -y - x + rx;
                pmotorBL = -y + x + rx;
                pmotorBR = -y - x - rx;
                pmotorFR = -y + x - rx;

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

                POWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEER(pmotorFR, pmotorFL, pmotorBR, pmotorBL);
            }
        }
    });



    private final Thread GARA = new Thread(new Runnable() {

        @Override
        public void run() {
            while(stop==false) {
                if(gamepad1.right_bumper){
                    var=0.5;
                }
                else{
                    var=0;
                }
                articulator1.setPower(var-gamepad1.right_trigger);
            }
        }
    });
        @Override
    public void loop() {

    }
}
