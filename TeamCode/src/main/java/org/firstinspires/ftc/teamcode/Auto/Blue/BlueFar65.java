package org.firstinspires.ftc.teamcode.Auto.Blue;

import static org.firstinspires.ftc.teamcode.opmode.ParasTeleop.ThresholdColor;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.TransferSeq;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Location;
import org.firstinspires.ftc.teamcode.hardware.PropPipeline;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
@Config
public class BlueFar65 extends LinearOpMode {

    private RobotHardware robot=RobotHardware.getInstance();   //Robot instance

    SampleMecanumDrive drive;
    //Subsytems
    IntakeSubsystem Intake=null;
    OutakeSubsystem Outtake=null;
    ElevatorSubsytem Elevator=null;
    ExtensionSubsystem Extension=null;
    public Location marker= Location.RIGHT;
    //ProPipeLine
    private PropPipeline propPipeline;
    private VisionPortal portal;

    public static int ThresholdDistance=15;
    public static int ThresholdColor=700;

    @Override
    public void runOpMode() throws InterruptedException {
        drive=new SampleMecanumDrive(hardwareMap);
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        Intake = new IntakeSubsystem(robot);
        Outtake = new OutakeSubsystem(robot);
        Elevator = new ElevatorSubsytem(robot);
        Extension = new ExtensionSubsystem(robot);
        robot.enabled = true;

        //REDO ROBOT INIT
        robot.flappers.setPosition(Globals.flapperClose);
        sleep(200);
        robot.stackServo.setPosition(Globals.stackInit);
        sleep(150);
        robot.Arm.setPosition(Globals.ArmInit);
        setServoShoulder(Globals.shoulderInit);
        robot.rotate.setPosition(Globals.rotateInit);
        robot.switchPixel.setPosition(Globals.switchPixelInit);
        Extension(0,1);


        PropPipeline propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        while (opModeInInit()){
            telemetry.addLine("ready");
            try{
                marker = propPipeline.getLocation();    //Storing PropLocation  ,if error throw outside loop
            }catch (Exception e){
                marker = Location.RIGHT;
            }
            // JOYSTICK //
            if(gamepad1.dpad_left){
                marker = Location.LEFT;
            }
            else if(gamepad1.dpad_up){
                marker = Location.CENTER;
            }
            else if(gamepad1.dpad_right){
                marker = Location.RIGHT;
            }
            telemetry.addData("marker",marker.toString());
            telemetry.update();
        }
        portal.close();




        Pose2d startPose = new Pose2d(-38, 62, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


//

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-56, 30, 60))
                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)

                // todo Write a command to drop purple pixel and take intake
                .addTemporalMarker(()-> Intake.rollOutside(0.75))
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.5)

                // todo going for purple pixel
                .lineToConstantHeading(new Vector2d(-56, 30))


                // todo write trajectories to the backdrop
                .addTemporalMarker(()-> detect())
                .lineToConstantHeading(new Vector2d(-38, 57))

                .lineToConstantHeading(new Vector2d(24, 57))

                // todo write command to drop yellow and white pixel
                // TRANSFER SEQUENCE
                .addTemporalMarker(() -> robot.intakeMotor.setPower(0))
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackDown))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperOpen))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Outtake.gripSafeOpen())
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
                .addTemporalMarker(()-> Outtake.rotatePick())
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmSafePick))
                .waitSeconds(0.25)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Outtake.ArmServo(Globals.ArmPick))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.3)
                .addTemporalMarker(()->Outtake.gripCloseBoth())
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(50,29))   // TODO (AT BACKDROP)

//                 TODO DROPPING THE YELLOW AND WHITE PIXEL

                // DROPPING SEQUENCE
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Intake.intakeStop())
//                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperOpen))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmSafeDrop))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .waitSeconds(2)
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .addTemporalMarker(()->Outtake.rotateDrop())
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.1)




//                 ================================================================= TODO FIRST CYCLE START FOR WHITE PIXEL ========================================================
                .lineToConstantHeading(new Vector2d(24, 55))

                // COMING BACK TO NEUTRAL POS
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackInit))
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmInit))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackInit))
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmInit))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.rotateInit())
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .addTemporalMarker(()-> Outtake.gripSafeOpen())

                .lineToConstantHeading(new Vector2d(-38, 55))

                // TODO TAKING PIXELS FROM STACK
                .addTemporalMarker(()-> Intake.rollOutside(0.75))
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.03)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-57.5, 40, Math.toRadians(60)))

                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-38, 55, 0))

                .lineToConstantHeading(new Vector2d(24, 55))

                .addTemporalMarker(() -> robot.intakeMotor.setPower(0))
                .lineToConstantHeading(new Vector2d(50,40))  // TODO (AT BACKDROP)

                // TODO DROPPING THE WHITE PIXEL ON THE BACKDROP

                // TRANSFER SEQUENCE
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackDown))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperOpen))
                .addTemporalMarker(()-> Outtake.gripSafeOpen())
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
                .addTemporalMarker(()-> Outtake.rotatePick())
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmSafePick))
                .waitSeconds(0.25)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Outtake.ArmServo(Globals.ArmPick))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Outtake.gripCloseBoth())

                // DROPPING SEQUENCE
                .addTemporalMarker(()-> Intake.intakeStop())
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperOpen))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmSafeDrop))
                .addTemporalMarker(()-> Outtake.rotatePreDrop())
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .addTemporalMarker(()->Outtake.rotateDrop())
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.1)


                // COMING BACK TO NEUTRAL POS
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackInit))
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmInit))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackInit))
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmInit))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .addTemporalMarker(()-> Outtake.rotateInit())
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .addTemporalMarker(()-> Outtake.gripSafeOpen())
                // ================================================================= TODO SECOND CYCLE START FOR WHITE PIXEL ========================================================
                .lineToConstantHeading(new Vector2d(24, 54))

                .lineToConstantHeading(new Vector2d(-38, 54))

                // TODO TAKING PIXELS FROM STACK
                .addTemporalMarker(()-> Intake.rollOutside(0.7))
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.03)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-59, 40, Math.toRadians(60)))

                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-38, 54, 0))

                .lineToConstantHeading(new Vector2d(24, 54))

                .addTemporalMarker(() -> robot.intakeMotor.setPower(0))
                .lineToConstantHeading(new Vector2d(50,40))  // TODO (AT BACKDROP)

                // TODO DROPPING THE WHITE PIXEL ON THE BACKDROP

                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackDown))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperOpen))
                .addTemporalMarker(()-> Outtake.gripSafeOpen())
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
                .addTemporalMarker(()-> Outtake.rotatePick())
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmSafePick))
                .waitSeconds(0.25)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Outtake.ArmServo(Globals.ArmPick))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Outtake.gripCloseBoth())


                // DROPPING SEQUENCE
                .addTemporalMarker(()-> Intake.intakeStop())
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperOpen))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmSafeDrop))
                .addTemporalMarker(()-> Outtake.rotatePreDrop())
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .addTemporalMarker(()->Outtake.rotateDrop())
                .addTemporalMarker(()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.1)

                // COMING BACK TO NEUTRAL POS
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackInit))
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmInit))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
                .addTemporalMarker(()-> Intake.setIntakeServo(Globals.stackInit))
                .addTemporalMarker(()-> Outtake.ArmServo(Globals.ArmInit))
                .addTemporalMarker(()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .addTemporalMarker(()-> Outtake.rotateInit())
                .addTemporalMarker(()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .addTemporalMarker(()-> Outtake.gripSafeOpen())
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-56, 30, -60))
                .build();



        waitForStart();

        if(marker==Location.LEFT){
            drive.followTrajectorySequence(left);
        } else if (marker==Location.RIGHT) {
            drive.followTrajectorySequence(right);
        }else {
            drive.followTrajectorySequence(center);
        }

        while (opModeIsActive()){
            telemetry.update();
        }



    }


    public void detect(){
        if(((robot.sensorColor1.red()>=ThresholdColor || robot.sensorColor1.blue()>=ThresholdColor || robot.sensorColor1.green()>=ThresholdColor ) && robot.sensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
                && ((robot.sensorColor2.red()>=ThresholdColor || robot.sensorColor2.blue()>=ThresholdColor || robot.sensorColor2.green()>=ThresholdColor) && robot.sensorColor2.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
            Intake.intakeStart(0.8);
            }
        }

    //Setting up lifter
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
