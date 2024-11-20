package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ElevatorSubsytem extends SubsystemBase {

    private RobotHardware robot;
    public ElevateState elevateState=ElevateState.HOME;

    public enum ElevateState{
        HOME,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        HANGERPOS,
        HANG

    }



    public ElevatorSubsytem(RobotHardware robot) {
        this.robot = robot;
    }


    public void updateState(ElevateState state,int DropConstant){
        this.elevateState=state;
        switch (state){
            case HOME:
                extendTo(Globals.lifterDown,1);
                break;
            case ONE:
                extendTo(Globals.lifterOne+DropConstant,1);
                break;
            case TWO:
                extendTo(Globals.lifterTwo+DropConstant,1);
                break;
            case THREE:
                extendTo(Globals.lifterThree+DropConstant,1);
                break;
            case FOUR:
                extendTo(Globals.lifterFour+DropConstant,1);
                break;
            case FIVE:
                extendTo(Globals.lifterFive+DropConstant,1);
                break;
            case SIX:
                extendTo(Globals.lifterSix+DropConstant,1);
                break;
            case SEVEN:
                extendTo(Globals.lifterSeven+DropConstant,1);
                break;
            case EIGHT:
                extendTo(Globals.lifterEight+DropConstant,1);
                break;
            case NINE:
                extendTo(Globals.lifterNine+DropConstant,1);
                break;
            case HANGERPOS:
                extendTo(Globals.hangpos, 1);
                break;
            case HANG:
                extendTo(Globals.hang, 1);
                break;
        }
}

public void extendTo(int targetPosition, double power){
    robot.leftElevator.setTargetPosition(targetPosition);
    robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftElevator.setPower(power);

    robot.rightElevator.setTargetPosition(targetPosition);
    robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightElevator.setPower(power);
}

public double[] getPosition(){
    return new double[]{robot.leftElevator.getCurrentPosition(), robot.rightElevator.getCurrentPosition()};
}


public void reset(){
    robot.leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}

public double[] getCurrent(){
    return new double[]{robot.leftElevator.getCurrent(CurrentUnit.AMPS), robot.rightElevator.getCurrent(CurrentUnit.AMPS)};
}
}