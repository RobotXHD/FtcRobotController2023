package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;

public class FunctiiDeAutonom extends LinearOpMode {
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
        while (turela.getCurrentPosition() < poz+1 && turela.getCurrentPosition() > poz-1){
            pid.setSetpoint(poz);
            pid_result = pid.performPID(poz);
            turela.setPower(pid_result);
        }
        pid.setSetpoint(poz);
        pid_result = pid.performPID(poz);
        turela.setPower(pid_result);
        while (turela.getCurrentPosition() < poz+1 && turela.getCurrentPosition() > poz-1){
            pid.setSetpoint(poz);
            pid_result = pid.performPID(poz);
            turela.setPower(pid_result);
        }
        pid.setSetpoint(poz);
        pid_result = pid.performPID(poz);
        turela.setPower(pid_result);
        while (turela.getCurrentPosition() < poz+1 && turela.getCurrentPosition() > poz-1){
            pid.setSetpoint(poz);
            pid_result = pid.performPID(poz);
            turela.setPower(pid_result);
        }
    }
    public void kdf(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime + t > System.currentTimeMillis()){

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
