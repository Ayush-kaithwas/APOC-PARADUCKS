package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "GetValues")
@Config
public class GetValues extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public double flaps=0.5; //flapper
    public static double intakeServo=0.5;
    public double shoulderpos=0.5;
    public double switchPix=0.5;

    public double rachetPos = 0.5;
    public double rotateP=0.5;
    public double leftGripPos=0.5;
    public double rightGripPos=0.5;
    public double ArmPos=0.5;
    public static double dronePos=0.5;
    public static double flapPOs=0.5;

    public static double power=0.8;
    public static int lifterPos=10;
    public static int hangerPos=0;
    public static int intakeMotorPos=0;
    public static int LeftElevatorPos=0;
    public static int RightElevatorPos=0;
    public static double offset=0.01;

    public static int ExtendPOs = 0;






    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


        robot.init(hardwareMap,telemetry);


        setServoShoulder(shoulderpos); // Both Shoulder
        robot.leftGrip.setPosition(leftGripPos); // Left Gripper
        robot.rightGrip.setPosition(rightGripPos); // Right Gripper
        setIntakeServo(Globals.stackDown); // Intake Servo


        robot.flappers.setPosition(flapPOs); // Flappers
        robot.switchPixel.setPosition(switchPix); // Switch Pixel
        robot.rotate.setPosition(rotateP); // Rotate
        robot.droneLock.setPosition(dronePos); // Drone
        robot.Arm.setPosition(ArmPos);
        robot.ratchet.setPosition(rachetPos);


    }

    @Override
    public void run() {
        super.run();

        // TODO===================================================== Rotate Servo (1,a,b) =================================================================
        if(gamepad1.a){
            rotateP+=0.001;
            robot.rotate.setPosition(rotateP);
        }
        else if (gamepad1.b) {
            rotateP-=0.001;
            robot.rotate.setPosition(rotateP);
        }
        // TODO===================================================== Shoulder Servo(1,dpad-left,right) =================================================================
        else if (gamepad1.dpad_left) {
            shoulderpos += 0.001;
            setServoShoulder(shoulderpos);
        }

        else if (gamepad1.dpad_right) {
            shoulderpos -= 0.001;
            setServoShoulder(shoulderpos);
        }

        // TODO===================================================== intake Servo =================================================================
        else if (gamepad1.left_bumper) {
            intakeServo+=0.001;
            setIntakeServo(intakeServo);
        }
        else if (gamepad1.right_bumper) {
            intakeServo-=0.001;
            setIntakeServo(intakeServo);
        }

        // TODO===================================================== Grip Servo =================================================================
        if (gamepad1.x) {
            leftGripPos+=0.001;
            robot.leftGrip.setPosition(leftGripPos);
        }
        else if (gamepad1.y) {
            leftGripPos-=0.001;
            robot.leftGrip.setPosition(leftGripPos);
        }
        else if (gamepad1.dpad_up) {
            rightGripPos+=0.001;
            robot.rightGrip.setPosition(rightGripPos);
        }
        else if (gamepad1.dpad_down) {
            rightGripPos-=0.001;
            robot.rightGrip.setPosition(rightGripPos);

        }

        // TODO===================================================== Elevator Motors  =================================================================
        else if (gamepad1.left_trigger>0) {
            extendTo(lifterPos,power);
        }
        else if (gamepad1.right_trigger>0) {
            extendTo(lifterPos,power);
        }

        if(gamepad2.dpad_up){
            lifterINC();
        } else if
        (gamepad2.dpad_down) {
            lifterDEC();
        }

        // TODO===================================================== Switch Pixel Servo =================================================================
        else if (gamepad1.back) {
            switchPix+=0.001;
            robot.switchPixel.setPosition(switchPix);
        }
        else if (gamepad1.start) {
            switchPix-=0.001;
            robot.switchPixel.setPosition(switchPix);
        }

        // TODO=====================================================Drone Servo =================================================================
        else if (gamepad2.dpad_left) {
            dronePos+=0.001;
            robot.droneLock.setPosition(dronePos);
        }
        else if (gamepad2.dpad_right) {
            dronePos-=0.001;
            robot.droneLock.setPosition(dronePos);
        }


        // TODO=====================================================Intake Motor =================================================================
//        else if (gamepad2.x)
//        {
////            intakeMotorPos+=10;
////            intakeMotorCounts(intakeMotorPos,1);
//            robot.intakeMotor.setPower(0.8);
//
//        }
//        else if (gamepad2.y)
//        {
////            intakeMotorPos-=10;
////            intakeMotorCounts(intakeMotorPos,1);
//            robot.intakeMotor.setPower(-0.8);
//        }
        else if(gamepad2.x){
            rachetPos +=0.001;
            robot.ratchet.setPosition(rachetPos);
        }else if(gamepad2.y){
            rachetPos -=0.001;
            robot.ratchet.setPosition(rachetPos);
        }

        // TODO===================================================== ARM Servo =================================================================
        else if (gamepad2.start) {
            ArmPos+=0.001;
            robot.Arm.setPosition(ArmPos);

        }else  if (gamepad2.back) {
            ArmPos-=0.001;
            robot.Arm.setPosition(ArmPos);

        } // TODO=====================================================Flapper Servo =================================================================
        else if (gamepad2.a) {
            flapPOs+=0.001;
            robot.flappers.setPosition(flapPOs);

        }else  if (gamepad2.b) {
            flapPOs-=0.001;
            robot.flappers.setPosition(flapPOs);
        }
        // TODO=====================================================  Extension X =================================================================


        else if (gamepad2.right_bumper) {
            Extension(-1600, 1);

        }
        else  if (gamepad2.left_bumper) {
            Extension(0, 1);

        }


        // TODO ===================================================== Rachet Servo ======================================================================
        else if (gamepad2.left_trigger>0) {
            rachetPos+=0.01;
            robot.ratchet.setPosition(rachetPos);
        }else if (gamepad2.right_trigger>0) {
            rachetPos-=0.01;
            robot.ratchet.setPosition(rachetPos);
        }


        telemetry.addData("Rotate", robot.rotate.getPosition());
        telemetry.addData("RAtCHET", robot.ratchet.getPosition());
        telemetry.addData("Flapper", robot.flappers.getPosition());
        telemetry.addData("Stack Servo", robot.stackServo.getPosition());
        telemetry.addData("Left Grip", robot.leftGrip.getPosition());
        telemetry.addData("Right Grip", robot.rightGrip.getPosition());
        telemetry.addData("Right Shoulder", robot.rightShoulder.getPosition());
        telemetry.addData("Arm", robot.Arm.getPosition());
        telemetry.addData("Left Shoulder", robot.leftShoulder.getPosition());
        telemetry.addData("switchPixel", robot.switchPixel.getPosition());
        telemetry.addData("DroneLock",robot.droneLock.getPosition());
        telemetry.addData("leftCurrent",robot.leftElevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightCurrent",robot.rightElevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lifterCountsL",robot.leftElevator.getCurrentPosition());
        telemetry.addData("Left Elevator POS",robot.leftElevator.getCurrentPosition());
        telemetry.addData("Right Elevator Pos",robot.rightElevator.getCurrentPosition());
        telemetry.addData("lifterCountsR",robot.rightElevator.getCurrentPosition());
        telemetry.addData("Extension Current",robot.IntakeExtensionLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Extension POS",robot.IntakeExtensionLeft.getCurrentPosition());

        telemetry.update();
    }

    public void setServoShoulder(double leftPos){    //Todo add servo offset if needed.
        double rightPos=1-leftPos;
        robot.leftShoulder.setPosition(leftPos);
        robot.rightShoulder.setPosition(rightPos);
    }

    public void extendTo(int targetPosition, double power){

        robot.leftElevator.setTargetPosition(targetPosition);
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition(targetPosition);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);
    }



    public void lifterINC()
    {
        robot.leftElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()+50);
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition(robot.rightElevator.getCurrentPosition()+50);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);

    }
    public void lifterDEC(){
        robot.leftElevator.setTargetPosition((robot.leftElevator.getCurrentPosition()-50));
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition((robot.leftElevator.getCurrentPosition()-50));
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);

    }

    //Setting up hanger
    public void executeHanger(int targetPosition,double power){
        robot.rightElevator.setTargetPosition(targetPosition);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);
    }

    public void intakeMotorCounts(int counts,double pow){
        robot.intakeMotor.setTargetPosition(counts);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeMotor.setPower(pow);
    }

    public void intakeMotorInc(){
        robot.intakeMotor.setTargetPosition( robot.intakeMotor.getCurrentPosition()+10);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeMotor.setPower(0.5);
    }
    public void intakeMotorDec(){
        robot.intakeMotor.setTargetPosition(robot.intakeMotor.getCurrentPosition()-10);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeMotor.setPower(0.5);
    }
    public void setIntakeServo(double IntakeServoPos){
        robot.stackServo.setPosition(IntakeServoPos);
    }

    public void Extension(int ExtendVal, double pow)
    {
        robot.IntakeExtensionLeft.setTargetPosition(ExtendVal);
        robot.IntakeExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.IntakeExtensionLeft.setPower(pow);
    }

    //////////////// TODO PICKING POSITIONS
    /*
    * Rotate = 0.5
    * Flapper = 0.54 // Change
    * Left Grip = 0.705
    * Right Grip = 0.5
    * ARM = 0.706
    * Switch Pixel = 0.4777
    * Right S  = 0.0116
    * Left S = 0.9877
    * */

    // GRAB THE PIXEL
//    Right GRIP  = 0.445
    // Left Grip = 0.879


    ////////////////// TODO DROP PIXEL
    /*
     * Rotate = 0.5
     * Flapper = 0.54 // Change
     * Left Grip = 0.705 // Change
     * Right Grip = 0.5 // Change
     * ARM = 0.30166
     * Switch Pixel = 0.480
     * Right S  = 0.5977
     * Left S = 0.40166
     * */




    /// TODO INatking the PIXEl









}

