package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {
    private RobotHardware robot;
    public IntakeServoState intakeServoState=IntakeServoState.INTAKE_UP;
    public RollerIntakeState rollerIntakeState=RollerIntakeState.INTAKE_OFF;
    public BeamState beamState=BeamState.BEAM_T;
    public FlappersState flappersState=FlappersState.FLAPPERS_INIT;
    public static double current;

    //States
    public enum IntakeServoState{  //stackServo
        INIT,
        INTAKE_DOWN,//TWO_PIXEL
        THREE_PIXEL,
        FIVE_PIXEL,
        INTAKE_UP,


    }

    public enum RollerIntakeState{ //intake motor
        PIXEL_IN,
        PIXEL_OUT,
        INTAKE_ON,
        INTAKE_OFF,
    }

    public enum BeamState{
        BEAM_T,
        GET_BEAM_1,
        GET_BEAM_2,
    }

    public enum FlappersState{
        FLAPPERS_INIT,
        FLAPPER_OPEN,
        FLAPPERS_MID,
        FLAPPER_CLOSE,

    }

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    //Intake stack servo
    public void updateState(IntakeServoState state){
        this.intakeServoState=state;
        switch (state){
            case INIT:
                setIntakeServo(Globals.stackInit);
                break;
            case INTAKE_DOWN:
                setIntakeServo(Globals.stackDown);
                break;
            case THREE_PIXEL:
                setIntakeServo(Globals.stackThree);
                break;
            case FIVE_PIXEL:
                setIntakeServo(Globals.stackFive);
                break;
        }
    }

    //Roller intake
    public void updateState(RollerIntakeState state){
        this.rollerIntakeState=state;
        switch(state){
            case INTAKE_ON:
                intakeStart(-0.7);//0.8
                break;
            case INTAKE_OFF:
                intakeStop();
                break;
            case PIXEL_OUT:
                rollOutside(-0.7);
                break;
        }
    }

    //Beam Break
//    public void updateState(BeamState state){
//        this.beamState=state;
//        switch (state){
//            case GET_BEAM_1:
//                getBeam1();
//                break;
//            case GET_BEAM_2:
//                getBeam2();
//                break;
//            case BEAM_T:
//                getBeam();
//                break;
//        }
//    }

    public void updateState(FlappersState state){
        this.flappersState=state;
        switch (state){
            case FLAPPER_OPEN:
                robot.flappers.setPosition(Globals.flapperOpen);
                break;
            case FLAPPERS_MID:
                robot.flappers.setPosition(Globals.flapperMid);
                break;
            case FLAPPER_CLOSE:
                robot.flappers.setPosition(Globals.flapperClose);
                break;
        }
    }

    public void intakeStart(double intakeSpeed){
        robot.intakeMotor.setPower(intakeSpeed);
    }
    public void rollOutside(double intakeSpeed){
        robot.intakeMotor.setPower(-intakeSpeed);
    }
    public void intakeStop(){
        robot.intakeMotor.setPower(0);
    }
    public double intakeCurrent(){
        return current=robot.intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    /* Beam Breaks */
//    public boolean getBeam1(){
//        return robot.beam1.getState();
//    }
//    public boolean getBeam2(){
//        return robot.beam2.getState();
//    }
//    public boolean[] getBeam(){
//        return  new boolean[]{ robot.beam1.getState(), robot.beam2.getState()};
//    }



    /* Flappers Servo */
    public void flapperOpen(){
        robot.flappers.setPosition(Globals.flapperOpen);
    }

    public void flapperMid(){
        robot.flappers.setPosition(Globals.flapperMid);
    }
    public void flapperClose(){
        robot.flappers.setPosition(Globals.flapperClose);
    }

    /* Intake/Stack servo */
    public void setIntakeServo(double IntakeServoPos){
        robot.stackServo.setPosition(IntakeServoPos);
    }


    public void intakeServoInit(){
        robot.stackServo.setPosition(0);
    }
    public void stackFive(){
        robot.stackServo.setPosition(Globals.stackFive);
    } public void stackFour(){
        robot.stackServo.setPosition(Globals.stackFour);
    }
    public void intakeServoUp(){
        robot.stackServo.setPosition(0);
    }
    public void intakeServoDown(){
        robot.stackServo.setPosition(0);
    }
    public void stackInit(){
        robot.stackServo.setPosition(Globals.stackInit);
    }

}
