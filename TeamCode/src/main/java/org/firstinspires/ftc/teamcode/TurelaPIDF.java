package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TurelaPIDFconstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Autonomous(name = "Calibrare Turela", group = "Calibrare")
public class TurelaPIDF extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx turela = hardwareMap.get(DcMotorEx.class, "motorTower");
        turela.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f));
        turela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turela.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turela.setPower(0);

        while(!isStarted() && !isStopRequested());

        while(!isStopRequested()){
            if(pidfChanged){
                turela.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f));
            }
            if(targetChanged){
                turela.setTargetPosition(targetPosition);
            }
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Position", turela.getCurrentPosition());
            telemetry.update();
        }
    }
}
