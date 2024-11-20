package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
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


public class Test extends CommandOpMode {

    public IntakeSubsystem intake = null;
    public OutakeSubsystem outtake = null;
    public ElevatorSubsytem elevator = null;

    public ExtensionSubsystem extension = null;

    public static int ElevatorPOs;
    public int counter =0;
    public int[] LifterStates={1,2,3,4,5,6,7,8,9};  //Use this for counter
    public int lifterCounter=1;
    public static int  droneShoot;
    public static  int droneCnt=0;
    public static boolean pixelDrop = FALSE;
    SampleMecanumDrive drive = null;
    private final RobotHardware robot = RobotHardware.getInstance();   //Robot instance
    public int ThresholdColor=700;
    public int ThresholdDistance=15;
    boolean PixelIntake = FALSE;
    public static int dropHeightOne = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeSubsystem(robot);
        outtake = new OutakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);
        extension = new ExtensionSubsystem(robot);

//        robot.flappers.setPosition(Globals.flapperClose);
//        sleep(200);
//        robot.stackServo.setPosition(Globals.stackInit);
//        sleep(150);
//        robot.Arm.setPosition(Globals.ArmInit);
//        setServoShoulder(Globals.shoulderInit);
//        robot.rotate.setPosition(Globals.rotateInit);
//        robot.switchPixel.setPosition(Globals.switchPixelInit);
        robot.droneLock.setPosition(Globals.DroneLock);
    }

    @Override
    public  void run()
    {
        super.run();
        if(gamepad2.start){
            schedule( new TransferSeq(intake, outtake, elevator));
        }
        telemetry.addData("red1-",robot.sensorColor1.red());
        telemetry.addData("blue1-",robot.sensorColor1.blue());
        telemetry.addData("green1-",robot.sensorColor1.green());
        telemetry.addData("red2-",robot.sensorColor2.red());
        telemetry.addData("blue2-",robot.sensorColor2.blue());
        telemetry.addData("green2-",robot.sensorColor2.green());
        telemetry.addData("distance1-",robot.sensorColor1.getDistance(DistanceUnit.MM));
        telemetry.addData("distance2-",robot.sensorColor2.getDistance(DistanceUnit.MM));

//        if(((robot.sensorColor1.red()>=ThresholdColor || robot.sensorColor1.blue()>=ThresholdColor || robot.sensorColor1.green()>=ThresholdColor ) && robot.sensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
//                && ((robot.sensorColor2.red()>=ThresholdColor || robot.sensorColor2.blue()>=ThresholdColor || robot.sensorColor2.green()>=ThresholdColor) && robot.sensorColor2.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
//            schedule(  new IntakePixel(intake, IntakeSubsystem.IntakeServoState.INTAKE_UP, IntakeSubsystem.RollerIntakeState.INTAKE_OFF));
//            schedule( new TransferSeq(intake, outtake, elevator));
//        }

        if(gamepad1.start){
            schedule(new IntakePosSeq(outtake, intake, elevator));
        }

//        if(gamepad1.b)
//        {
//            schedule(  new IntakePixel(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_ON)); // Intaking the Pixel From ground
//        }
//        else if (gamepad1.a)
//        {
//            schedule(  new IntakePixel(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_OFF));
//        }
//        else if (gamepad1.right_trigger>0) {
//            schedule(  new IntakePixel(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.PIXEL_OUT)); // Intaking the Pixel From ground
//
//        } else if(gamepad1.left_bumper){
//            schedule(new DropSeq(intake,outtake));
//        }
//        else if(gamepad1.x){
//            robot.flappers.setPosition(Globals.flapperClose);
//        }
//        else if(gamepad1.y){
//            schedule(new GripperCommand(outtake, OutakeSubsystem.GripperState.GRIP_OPEN));
//        }
//        else if(gamepad1.right_bumper)
//        {
//            robot.stackServo.setPosition(Globals.stackFive);
//        }
//        else if(gamepad1.left_trigger>0){
//            robot.stackServo.setPosition(Globals.stackFour);
//        }
//        else if(gamepad1.start){
//            robot.stackServo.setPosition(Globals.stackThree);
//        }
//        else if(gamepad1.back){
//            robot.stackServo.setPosition(Globals.stackDown);
//        }
//
        else if(gamepad1.dpad_up)
        {
            inc(0.7);
        }
        else if(gamepad1.dpad_down){
            dec(0.7);
        }
//
//        else if(gamepad2.left_bumper){
//            schedule( new ExtensionCommand(extension, ExtensionSubsystem.IntakeExtensionState.INIT));
//        }
//        else if(gamepad2.right_bumper){
//            schedule( new ExtensionCommand(extension, ExtensionSubsystem.IntakeExtensionState.Extend));
//        }
        if(gamepad2.x)
        {
            // Hang position
            schedule(new DropSeq(intake,outtake));
            sleep(500);
            schedule(new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HANGERPOS,dropHeightOne));
        }
        if(gamepad2.a)
        {
            // Hang
            schedule( new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HANG, dropHeightOne));
        }

        if(gamepad2.right_bumper){
            robot.ratchet.setPosition(Globals.RachetClose);
        }
        else if(gamepad2.left_bumper){
            robot.ratchet.setPosition(Globals.RachetOpen);
    }

        else if(gamepad2.y){
            robot.droneLock.setPosition(Globals.DroneOPen);
        }







        // TODO ============================================ Drive ===========================================================
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("Left Motor POS", robot.leftElevator.getCurrentPosition());
        telemetry.addData("right Motor POS", robot.rightElevator.getCurrentPosition());
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

    public void inc(double power){

        robot.leftElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()-10);
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()-10);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);
    }


    public void dec(double power){

        robot.leftElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()+10);
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()+10);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);
    }


}
