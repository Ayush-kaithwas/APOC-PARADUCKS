package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.RotateCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.SwitchPixelCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.DropSeq;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.IntakePixel;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.IntakePosSeq;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.TransferSeq;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.security.cert.Extension;

@TeleOp
@Config


public class ParasTeleop extends CommandOpMode {

    public IntakeSubsystem Intake = null;
    public OutakeSubsystem Outtake = null;
    public ElevatorSubsytem Elevator = null;

    public ExtensionSubsystem Extension = null;

    public static int ElevatorPOs;
    public int counter =0;
    public int[] LifterStates={1,2,3,4,5,6,7,8,9};  //Use this for counter
    public int lifterCounter=0;
    public static int  droneShoot;
    public static  int droneCnt=0;
    public static boolean pixelDrop = false;
    SampleMecanumDrive drive = null;
    private final RobotHardware robot = RobotHardware.getInstance();   //Robot instance

    boolean PixelIntake = false;
    public static int dropHeightOne = 0;
    public static int ThresholdDistance=15;
    public static int ThresholdColor=700;
    public static boolean extensionFlag = false;
    public GamepadEx gamepadEx;
    public GamepadEx gamepadEx2;
    public boolean extendit = false;
    public boolean isIntake = false;
    public boolean isDrop = false;


    public static double head = 0.8;
    public static double straight = 0.8;
    public static double strafe = 0.8;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        Intake = new IntakeSubsystem(robot);
        Outtake = new OutakeSubsystem(robot);
        Elevator = new ElevatorSubsytem(robot);
        Extension = new ExtensionSubsystem(robot);
        gamepadEx=new GamepadEx(gamepad1);

        gamepadEx2 = new GamepadEx(gamepad2);


        //// TODO ================================================== INIT ================================================

        robot.flappers.setPosition(Globals.flapperClose);
        sleep(200);
        robot.stackServo.setPosition(Globals.stackInit);
        sleep(150);
        robot.Arm.setPosition(Globals.ArmInit);
        sleep(200);
        setServoShoulder(Globals.shoulderInit);
        robot.rotate.setPosition(Globals.rotateInit);
        robot.switchPixel.setPosition(Globals.switchPixelInit);

        while(opModeInInit())
        {
            Extension(0,1);
        }

        // TODO ============================================ Single Pixel Drop ===========================================================

       //// TODO Toggle Logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_LEFT_OPEN),
                new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_RIGHT_OPEN)
        );

        // TODO ============================================= PIXEL OUT AND OFF TOGGLE =====================================================
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_UP, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_UP, IntakeSubsystem.RollerIntakeState.PIXEL_OUT)
        );



    }

    @Override
    public  void run()
    {
        super.run();

        // TODO ============================================ Intake Position ===========================================================
        if(gamepad1.right_bumper && !isDrop){
            head = 0.8;
            strafe = 0.8;
            straight = 0.8;
            schedule( new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_OPEN));
            sleep(200);
            schedule( new IntakePosSeq(Outtake, Intake,Elevator));
            extendit = FALSE;
            isIntake = FALSE;
            isDrop = true;
        }


        // TODO ============================================ Ground Level Intake ===========================================================
        if(gamepad1.b){
            schedule(  new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_ON)); // Intaking the Pixel From ground
            head = 0.5;
            strafe = 0.5;
            straight = 0.5;
        }

       // TODO ============================================ Outtake ===========================================================
      // Transferring the pixel when colour sensor gets active
        if(((robot.sensorColor1.red()>=ThresholdColor || robot.sensorColor1.blue()>=ThresholdColor || robot.sensorColor1.green()>=ThresholdColor ) && robot.sensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
                && ((robot.sensorColor2.red()>=ThresholdColor || robot.sensorColor2.blue()>=ThresholdColor || robot.sensorColor2.green()>=ThresholdColor) && robot.sensorColor2.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
            if(extensionFlag || robot.IntakeExtensionLeft.getCurrentPosition()<5){
                schedule( new TransferSeq(Intake, Outtake, Elevator));
            }
            head = 0.8;
            strafe = 0.8;
            straight = 0.8;
            isDrop = true;
        }

        //////// Transferring logic ends
        if(gamepad2.start) // Forcefully Executing the Transfer Sequence
        {
            schedule(new TransferSeq(Intake, Outtake, Elevator));
            head = 0.8;
            strafe = 0.8;
            straight = 0.8;
            isDrop = true;
        }

        if(gamepad1.left_bumper && isDrop)
        {
            head = 0.5;
            strafe = 0.5;
            straight = 0.5;

            schedule(new DropSeq(Intake,Outtake));
            extendit = TRUE;
            isIntake = TRUE;
            isDrop = FALSE;

        }


        // TODO ============================================ Both Pixel Drop ===========================================================
        if(gamepad1.right_trigger>0)
        {
            schedule( new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_OPEN));
        }

        // TODO ============================================ Reverse Intake ===========================================================
        if(gamepad1.left_trigger>0)
        {
            Intake.rollOutside(0.8);
        }

//        // TODO ============================================ Drone Shoot ===========================================================
//        if(gamepad1.back)
//        {
//            // Press back double time
//            if(droneCnt>1)
//            {
//                robot.droneLock.setPosition(droneShoot);
//            }
//            else {
//                droneCnt++;
//            }
//        }
//

        // TODO ================================================= STACK PIXEL ========================================================
        if(gamepad2.a){
            schedule( new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.FIVE_PIXEL));
            schedule(  new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_ON));

        }

//
//        // TODO ============================================ Hanging ===========================================================
        if(gamepad1.x)
        {
            // Hang position
            schedule( new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HANGERPOS, dropHeightOne));
        }
        if(gamepad1.a)
        {
            // Hang
            schedule( new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HANG, dropHeightOne));
        }

        // TODO ============================================ X Extension  ===========================================================
        if(gamepad1.x)
        {
            // Hang position
            schedule( new ExtensionCommand(Extension, ExtensionSubsystem.IntakeExtensionState.INIT));
            sleep(2000);
            extensionFlag = TRUE;
        }
        if(gamepad1.a)
        {
            // Hang
            schedule( new ExtensionCommand(Extension, ExtensionSubsystem.IntakeExtensionState.Extend));
            extensionFlag = FALSE;
        }
        //
//        // TODO ============================================ Elevator Height  ===========================================================
        if(gamepad2.dpad_up){
            if(lifterCounter<9)
            {
                lifterCounter++;
            }
        }
        if(gamepad2.dpad_down){
            if(lifterCounter>1)
            {
                lifterCounter--;
            }
        }

        if(gamepad1.dpad_up && extendit == TRUE)
        {
           if(lifterCounter>=1){
               if (LifterStates[lifterCounter - 1] == 1) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.ONE,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 2) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.TWO,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 3) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.THREE,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 4) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.FOUR,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 5) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.FIVE,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 6) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.SIX,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 7) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.SEVEN,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 8) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.EIGHT,dropHeightOne));
               }
               else if (LifterStates[lifterCounter - 1] == 9) {
                   schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.NINE,dropHeightOne));
               }
           }
        }

        // TODO ============================================ Drive ===========================================================
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y *  straight,
                        -gamepad1.left_stick_x * strafe,
                        -gamepad1.right_stick_x * head
                )
        );
        telemetry.addData("Heading:",drive.robotHeading);


        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("Left Motor POS", robot.leftElevator.getCurrentPosition());
        telemetry.addData("Left Motor POS", robot.rightElevator.getCurrentPosition());
        telemetry.addData("red1-",robot.sensorColor1.red());
        telemetry.addData("blue1-",robot.sensorColor1.blue());
        telemetry.addData("green1-",robot.sensorColor1.green());
        telemetry.addData("red2-",robot.sensorColor2.red());
        telemetry.addData("blue2-",robot.sensorColor2.blue());
        telemetry.addData("green2-",robot.sensorColor2.green());
        telemetry.addData("Height" , lifterCounter);
        telemetry.addData("distance1-",robot.sensorColor1.getDistance(DistanceUnit.MM));
        telemetry.addData("distance2-",robot.sensorColor2.getDistance(DistanceUnit.MM));
        telemetry.update();

    }

    public void setServoShoulder(double leftPos){    //Todo add servo offset if needed.
        double rightPos=1-leftPos;
        robot.leftShoulder.setPosition(leftPos);
        robot.rightShoulder.setPosition(rightPos);
    }

    public void Extension(int ExtendVal, double pow)
    {
        robot.IntakeExtensionLeft.setTargetPosition(ExtendVal);
        robot.IntakeExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.IntakeExtensionLeft.setPower(pow);
    }


}
