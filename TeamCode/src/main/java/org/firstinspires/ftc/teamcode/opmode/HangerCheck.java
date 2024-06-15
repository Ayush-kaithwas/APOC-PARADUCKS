package org.firstinspires.ftc.teamcode.systemcheck;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class HangerCheck extends LinearOpMode {

    public DcMotorEx hangerMotor1=null;
    public DcMotorEx hangerMotor2=null;

    public static int targetPos=0;
    public static double powerON=1;

    //-2409 up

    @Override
    public void runOpMode() throws InterruptedException {

        hangerMotor1=hardwareMap.get(DcMotorEx.class, "M1");
        hangerMotor2=hardwareMap.get(DcMotorEx.class, "M2");

        while (opModeInInit()){
            hangerMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangerMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.dpad_up){
                executeHanger(targetPos,1);
            } else if (gamepad1.dpad_down) {
                executeHanger(-targetPos,1);
            }

            /////////////////////////////////////
            if(gamepad1.a){
                hangerMotor1.setPower(powerON);
                hangerMotor2.setPower(powerON);

            } else if (gamepad1.b) {
                hangerMotor1.setPower(0);
                hangerMotor2.setPower(0);
            }
            //////////////////////////////////////
            if(gamepad1.x){
                hangerMotor1.setPower(-powerON);
                hangerMotor2.setPower(-powerON);
            }
            telemetry.addData("M1",hangerMotor1.getCurrentPosition());
            telemetry.addData("M1 Current",hangerMotor1.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("                                 ");
            telemetry.addData("M2",hangerMotor2.getCurrentPosition());
            telemetry.addData("M2 Current",hangerMotor2.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }

    public void executeHanger(int targetPosition,double power){
        hangerMotor1.setTargetPosition(targetPosition);
        hangerMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor1.setPower(power);

        hangerMotor2.setTargetPosition(targetPosition);
        hangerMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor2.setPower(power);
    }
}
