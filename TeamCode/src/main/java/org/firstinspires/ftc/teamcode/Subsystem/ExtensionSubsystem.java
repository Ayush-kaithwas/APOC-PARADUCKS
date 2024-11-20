package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.security.cert.Extension;

public class ExtensionSubsystem extends SubsystemBase {
    private RobotHardware robot;
    public static double current;
    private ExtensionSubsystem.IntakeExtensionState IntakeExtensionState;

    //States
    public enum IntakeExtensionState{  //Extension Intake
        INIT,
        Extend,
        PULL_IN
    }



    public ExtensionSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    //Intake stack servo
    public void updateState(IntakeExtensionState state){
        this.IntakeExtensionState=state;
        switch (state){
            case INIT:
                Extension(Globals.ExtendInit, 1);
                break;
            case Extend:
                Extension(Globals.Extend, 1);
                break;
            case PULL_IN:
                Extension(Globals.Pull_In, 1);

        }
    }

    public void Extension(int ExtendVal, double pow)
    {
        robot.IntakeExtensionLeft.setTargetPosition(ExtendVal);
        robot.IntakeExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.IntakeExtensionLeft.setPower(pow);
    }
    public void reset(){
        robot.IntakeExtensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
