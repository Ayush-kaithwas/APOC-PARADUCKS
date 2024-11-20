package org.firstinspires.ftc.teamcode.Subsystem;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class OutakeSubsystem {
    private RobotHardware robot;
    ShoulderState shoulderStatee=ShoulderState.INIT;
    //SwitchPixel
    public SwitchPixelState switchPixelStatee=SwitchPixelState.SWITCH_INIT;
    public SwitchPixelState switchPixelLeft=SwitchPixelState.SWITCH_LEFT;
    public SwitchPixelState switchPixelRight=SwitchPixelState.SWITCH_RIGHT;
    public SwitchPixelState switchPixelHorizontal=SwitchPixelState.SWITCH_HORIZONTAL;
    public SwitchPixelState switchPixelVertical=SwitchPixelState.SWITCH_VERTICAL;
    ////////////
    public GripperState gripperState=GripperState.GRIP_CLOSE;
    RotateState rotateState=RotateState.INIT;
    public OutakeState outakeState=OutakeState.LOW;
    /* Setting up States */
    public static String shoulderState="INIT";
    public static int shoulderStateID=0;

    public static String switchPixelState="INIT";

    public  ArmState armState = ArmState.INIT;
    public enum OutakeState{
        HIGH,
        LOW
    }
    public enum ShoulderState{
        PICK,
        SAFEPICK,
        SAFEPICK1,
        SAFEPICK2,
        PREPICK,
        DROP,
        SAFEDROP,
        PREDROP,
        INIT,
    }

    public enum SwitchPixelState{
        SWITCH_LEFT,
        SWITCH_LEFT_INVERSE,
        SWITCH_RIGHT,
        SWITCH_RIGHT_INVERSE,
        SWITCH_HORIZONTAL,
        SWITCH_HORIZONTAL_INVERSE,
        SWITCH_VERTICAL,
        SWITCH_VERTICAL_INVERSE,
        SWITCH_INIT,
        SWITCH_DROP,
        SWITCH_PIXEL_INTAKE
    }

    public enum GripperState{
        GRIP_LEFT_OPEN,
        GRIP_LEFT_CLOSE,
        GRIP_RIGHT_OPEN,
        GRIP_RIGHT_CLOSE,
        GRIP_OPEN,
        GRIP_CLOSE,
        GRIP_SAFE
    }

    public enum RotateState{
        INIT,
        PICK,
        MID,
        PLACE,
        PREDROP
    }

    public enum ArmState{
        INIT,
        PICK,
        SAFEPICK,
        PREPICK,
        MID,
        DROP,
        SAFEDROP,
        PREDROP,
    }



    public OutakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    //ShoulderState
    public void updateState(ShoulderState state){
        this.shoulderStatee=state;
        switch (state){
            case INIT:
                outakeState=OutakeState.LOW;
                setServoShoulder(Globals.shoulderInit);
                break;
            case PICK:
                outakeState=OutakeState.LOW;
                setServoShoulder(Globals.shoulderPick);
                break;
            case SAFEPICK:
                outakeState=OutakeState.LOW;
                setServoShoulder(Globals.shoulderSafePick);
                break;
                case SAFEPICK1:
                outakeState=OutakeState.LOW;
                setServoShoulder(Globals.shoulderSafePick1);
                break;
                case SAFEPICK2:
                outakeState=OutakeState.LOW;
                setServoShoulder(Globals.shoulderSafePick2);
                break;
            case PREPICK:
                outakeState=OutakeState.LOW;
                setServoShoulder(Globals.shoulderPrePick);
                break;
            case DROP:
                outakeState=OutakeState.HIGH;
                setServoShoulder(Globals.shoulderDrop);
                break;
            case SAFEDROP:
                outakeState=OutakeState.HIGH;
                setServoShoulder(Globals.shoulderSafeDrop);
                break;
            case PREDROP:
                outakeState=OutakeState.HIGH;
                setServoShoulder(Globals.shoulderPreDrop);
                break;
        }
    }

    public void updateState(ArmState state){
        this.armState=state;
        switch (state){
            case INIT:
                ArmServo(Globals.ArmInit);
                break;
            case PICK:
                ArmServo(Globals.ArmPick);
                break;
            case SAFEPICK:
                ArmServo(Globals.ArmSafePick);
                break;
            case PREPICK:
                ArmServo(Globals.ArmPrePick);
                break;
            case MID:
                ArmServo(Globals.ArmMid);
                break;
            case DROP:
                ArmServo(Globals.ArmDrop);
                break;
            case SAFEDROP:
                ArmServo(Globals.ArmSafeDrop);
                break;
            case PREDROP:
                ArmServo(Globals.ArmPreDrop);
                break;
        }
    }

    //SwitchPixel
    public void updateState(SwitchPixelState state){
        this.switchPixelStatee=state;
        this.switchPixelLeft=state;
        this.switchPixelRight=state;
        this.switchPixelHorizontal=state;
        this.switchPixelVertical=state;

        switch (state){
            case SWITCH_INIT:
                setSwitchPixel(Globals.switchPixelInit);
                break;
            case SWITCH_DROP:
                setSwitchPixel(Globals.switchPixelDrop);
                break;
            case SWITCH_HORIZONTAL:
                setSwitchPixel(Globals.switchPixelHorizontal);
                break;
            case SWITCH_HORIZONTAL_INVERSE:
                setSwitchPixel(Globals.switchPixelHorizontalInverse);
                break;
            case SWITCH_VERTICAL:
                setSwitchPixel(Globals.switchPixelVertical);
                break;
            case SWITCH_VERTICAL_INVERSE:
                setSwitchPixel(Globals.switchPixelVerticalInverse);
                break;
            case SWITCH_LEFT:
                setSwitchPixel(Globals.switchPixelLeft);
                break;
            case SWITCH_LEFT_INVERSE:
                setSwitchPixel(Globals.switchPixelLeftInverse);
                break;
            case SWITCH_RIGHT:
                setSwitchPixel(Globals.switchPixelRight);
                break;
            case SWITCH_RIGHT_INVERSE:
                setSwitchPixel(Globals.switchPixelRightInverse);
                break;
        }
    }

    //SwitchPixel
    public void updateState(SwitchPixelState state,double customPos){
        this.switchPixelStatee=state;
        this.switchPixelLeft=state;
        this.switchPixelRight=state;
        this.switchPixelHorizontal=state;
        this.switchPixelVertical=state;

        switch (state){
            case SWITCH_INIT:
                setSwitchPixel(Globals.switchPixelInit);
                break;
            case SWITCH_HORIZONTAL:
                setSwitchPixel(Globals.switchPixelHorizontal);
                break;
            case SWITCH_HORIZONTAL_INVERSE:
                setSwitchPixel(Globals.switchPixelHorizontalInverse);
                break;
            case SWITCH_VERTICAL:
                setSwitchPixel(Globals.switchPixelVertical);
                break;
            case SWITCH_VERTICAL_INVERSE:
                setSwitchPixel(Globals.switchPixelVerticalInverse);
                break;
            case SWITCH_LEFT:
                setSwitchPixel(Globals.switchPixelLeft);
                break;
            case SWITCH_LEFT_INVERSE:
                setSwitchPixel(Globals.switchPixelLeftInverse);
                break;
            case SWITCH_RIGHT:
                setSwitchPixel(Globals.switchPixelRight);
                break;
            case SWITCH_RIGHT_INVERSE:
                setSwitchPixel(Globals.switchPixelRightInverse);
                break;
            case SWITCH_PIXEL_INTAKE:
                setSwitchPixel(Globals.switchPixelIntake);
                break;
        }
    }

    //Gripper
    public void updateState(GripperState state){
        this.gripperState=state;
        switch (state){
            case GRIP_OPEN:                           //BOTH
                gripOpenBoth();
                break;
            case GRIP_CLOSE:
                gripCloseBoth();
                break;
            /////////////////////////////
            case GRIP_LEFT_OPEN:
                leftGripOpen();
                break;
            case GRIP_LEFT_CLOSE:
                leftGripClose();
                break;
            /////////////////////////////
            case GRIP_RIGHT_OPEN:
                rightGripOpen();
                break;
            case GRIP_RIGHT_CLOSE:
                rightGripClose();
                break;
            case GRIP_SAFE:
                gripSafeOpen();
                break;
        }
    }

    //ROTATE
    public void updateState(RotateState state){
        this.rotateState=state;
        switch (state){
            case INIT:
                rotateInit();
                break;
            case PICK:
                rotatePick();
                break;
            case MID:
                rotateMid();
                break;
            case PLACE:
                rotateDrop();
                break;
            case PREDROP:
                rotatePreDrop();
                break;
        }
    }

    /**
     * Shoulder Servo
     */
    public void setServoShoulder(double leftPos){    //Todo add servo offset if needed.
        double rightPos=1-leftPos;
        robot.leftShoulder.setPosition(leftPos);
        robot.rightShoulder.setPosition(rightPos);
    }

    public void shoulderPick(){
        shoulderState="PICK";
        shoulderStateID=0;
        setServoShoulder(Globals.shoulderPick);
    }
    public void shoulderDrop(){
        shoulderState="DROP";
        shoulderStateID=1;
        setServoShoulder(Globals.shoulderDrop);
    }
    public void shoulderInit(){
        shoulderState="INIT";
        shoulderStateID=2;
        setServoShoulder(Globals.shoulderInit);
    }

    /**
     * Arm Function
     */

    public void ArmServo(double pos){
        robot.Arm.setPosition(pos);
    }


    /**
     * Switch Pixel
     */

    public void setSwitchPixel(double pixelPos){   //Custom method
        robot.switchPixel.setPosition(pixelPos);
    }
    public void switchPixelLeft(){
        robot.switchPixel.setPosition(Globals.switchPixelLeft);
    }
    public void switchPixelRight(){
        robot.switchPixel.setPosition(Globals.switchPixelRight);
    }

    /**
     * Gripper
     */

    public void setLeftGrip(double leftGripPos){robot.leftGrip.setPosition(leftGripPos);}
    public void setRightGrip(double rightGripPos){
        robot.rightGrip.setPosition(rightGripPos);
    }

    public void setGrip(double leftGrip,double rightGrip){
        robot.leftGrip.setPosition(leftGrip);
        robot.rightGrip.setPosition(rightGrip);
    }
    public void gripOpenBoth(){
        robot.leftGrip.setPosition(Globals.leftGripOpen);
        robot.rightGrip.setPosition(Globals.rightGripOpen);
    }
     public void gripSafeOpen(){
        robot.leftGrip.setPosition(Globals.leftGripSafe);
        robot.rightGrip.setPosition(Globals.rightGripSafe);
    }



    public void gripCloseBoth(){
        robot.leftGrip.setPosition(Globals.leftGripClose);
        robot.rightGrip.setPosition(Globals.rightGripClose);
    }

    //Left
    public void leftGripOpen(){
        robot.leftGrip.setPosition(Globals.leftGripOpen);
    }

    public void leftGripSafe(){
        robot.leftGrip.setPosition(Globals.leftGripSafe);//mid
    }

    public void leftGripClose(){robot.leftGrip.setPosition(Globals.leftGripClose);}

    //Right
    public void rightGripOpen(){
        robot.rightGrip.setPosition(Globals.rightGripOpen);
    }
    public void rightGripSafe(){robot.rightGrip.setPosition(Globals.rightGripSafe); //mid
    }
    public void rightGripClose(){
        robot.rightGrip.setPosition(Globals.rightGripClose);
    }


    /**
     * Rotate Servo
     */

    public void setRotate(double rotatePos){
        robot.rotate.setPosition(rotatePos);
    }

    public void rotateInit(){
        robot.rotate.setPosition(Globals.rotateInit);
    }
    public void rotatePreDrop(){
        robot.rotate.setPosition(Globals.rotatePreDrop);
    }
    public void rotatePick(){
        robot.rotate.setPosition(Globals.rotatePick);
    }
    public void rotateMid(){
        robot.rotate.setPosition(Globals.rotateInit);
    }

    public void rotateDrop(){
        robot.rotate.setPosition(Globals.rotateDrop);
    }

}
