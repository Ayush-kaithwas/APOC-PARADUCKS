package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class HangerTesting extends LinearOpMode {

    public static int hangingPOS;
    public static int hang;

    DcMotorEx LeftHangerMotor;
    DcMotorEx RightHangerMotor;




    @Override
    public void runOpMode() throws InterruptedException {

        LeftHangerMotor = hardwareMap.get(DcMotorEx.class, "LHanger");
        RightHangerMotor = hardwareMap.get(DcMotorEx.class, "RHanger");




        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                LeftHangerMotor.setTargetPosition(hangingPOS);
                LeftHangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftHangerMotor.setPower(0.8);


                RightHangerMotor.setTargetPosition(hangingPOS);
                RightHangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightHangerMotor.setPower(0.8);
            }
            if(gamepad1.b){
                LeftHangerMotor.setTargetPosition(hang);
                LeftHangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftHangerMotor.setPower(0.8);

                RightHangerMotor.setTargetPosition(hang);
                RightHangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightHangerMotor.setPower(0.8);
            }
        }

    }
}
