package org.firstinspires.ftc.teamcode.Auto.Blue;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
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
public class BlueFar65Unstabble extends LinearOpMode {

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

        // REDO ROBOT INIT
        robot.flappers.setPosition(Globals.flapperClose);
        sleep(200);
        robot.stackServo.setPosition(Globals.stackInit);
        sleep(150);
        robot.Arm.setPosition(Globals.ArmInit);
        setServoShoulder(Globals.shoulderInit);
        robot.rotate.setPosition(Globals.rotateInit);
        robot.switchPixel.setPosition(Globals.switchPixelInit);
        robot.ratchet.setPosition(Globals.RachetOpen);

        // INIT POS
        inIt();

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
            }
            catch (Exception e) {
                marker = Location.CENTER;
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


        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(-41, 20))

                // todo going for purple pixel
                .lineToConstantHeading(new Vector2d(-51, 23))  // Dropping Purple Pixel

                // TODO DROPPING THE PURPLE PIXEL
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->Outtake.setServoShoulder(0.14))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()->Elevator.extendTo(-50, 0.8)) // -419
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(0.3788))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.gripOpenBoth())
                .waitSeconds(0.1)

                // todo Write a command to start intake
                .lineToConstantHeading(new Vector2d(-53, 34.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Elevator.extendTo(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.3)




                // RE PLUNGING
                .lineToConstantHeading(new Vector2d(-45, 45))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.5))
                .waitSeconds(0.1)

                .lineToConstantHeading(new Vector2d(-53, 33))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Elevator.extendTo(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

                // todo write trajectories to the backdrop
                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-45, 53, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))
                .lineToLinearHeading(new Pose2d(-35, 57.001, 0))

                .lineToConstantHeading(new Vector2d(24, 57))
                // TRANSFER SEQUENCE


                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.intakeStop())
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> robot.flappers.setPosition(Globals.flapperOpen))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.gripSafeOpen())
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Elevator.extendTo(Globals.lifterDown,1))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotatePick())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(Globals.ArmSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.ArmServo(Globals.ArmPick))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->Outtake.gripCloseBoth())


                .lineToConstantHeading(new Vector2d(48,33))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(Globals.ArmDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelRight))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterOne,1))


// TODO ========================== DROPPING THE YELLOW AND WHITE PIXEL ==========================================================

                .lineToConstantHeading(new Vector2d(52,29))   // TODO (AT BACKDROP)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.02)
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(0,1))




// TODO ================================================================= TODO FIRST CYCLE START FOR WHITE PIXEL ========================================================
                .lineToConstantHeading(new Vector2d(24, 55))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> Intake.rollOutside(0.9))


                // COMING BACK TO NEUTRAL POS
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.gripSafeOpen())

                .lineToConstantHeading(new Vector2d(-37, 55))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))

                // YELLOW AND WHITE DROPPED
                // TODO TAKING PIXELS FROM STACK
                .lineToLinearHeading(new Pose2d(-59, 40, Math.toRadians(60))) // TODO STACK POSE
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

                .lineToLinearHeading(new Pose2d(-38, 57, 0))
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))

                .lineToConstantHeading(new Vector2d(24, 57))

                // TRANSFER SEQUENCE
//                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> Intake.intakeStart(0.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.intakeStop())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> robot.flappers.setPosition(Globals.flapperOpen))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.gripSafeOpen())
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Elevator.extendTo(Globals.lifterDown,1))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotatePick())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(Globals.ArmSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.ArmServo(Globals.ArmPick))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->Outtake.gripCloseBoth())


                // todo write command to drop yellow and white pixel5
                .lineToConstantHeading(new Vector2d(48,35))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(Globals.ArmDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterTwo,1))

                .lineToConstantHeading(new Vector2d(52,32))  // TODO (AT BACKDROP SECOND CALL)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.02)

                .lineToConstantHeading(new Vector2d(40,32))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))


                .lineToConstantHeading(new Vector2d(45,50))        // TODO (PARKING)
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackInit))

                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.gripSafeOpen())
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)

                //
                .lineToConstantHeading(new Vector2d(-50, 30))

                // todo going for purple pixel
                .lineToLinearHeading(new Pose2d(-53, 40, Math.toRadians(-60)))
                // TODO DROPPING THE PURPLE PIXEL
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->Outtake.setServoShoulder(0.30))
                .waitSeconds(0.01)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->Outtake.setServoShoulder(0.10))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Outtake.ArmServo(0.323))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Outtake.setSwitchPixel(0.2339))
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.05)

                .lineToConstantHeading(new Vector2d(-54, 55))



                .lineToLinearHeading(new Pose2d(-54, 40, Math.toRadians(45)))
                // todo Write a command to drop purple pixel and take intake
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.3)


                // rePlunging
                .lineToConstantHeading(new Vector2d(-45, 45))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Intake.intakeStart(0.5))





                .lineToConstantHeading(new Vector2d(-55, 40))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Elevator.extendTo(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)


                // todo write trajectories to the backdrop

                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-45, 53, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))
                .lineToLinearHeading(new Pose2d(-35, 57.001, 0))


                .lineToConstantHeading(new Vector2d(24, 57))
                // TRANSFER SEQUENCE


                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.intakeStop())
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> robot.flappers.setPosition(Globals.flapperOpen))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.gripSafeOpen())
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Elevator.extendTo(Globals.lifterDown,1))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotatePick())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(Globals.ArmSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.ArmServo(Globals.ArmPick))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0,()->Outtake.gripCloseBoth())


                .lineToConstantHeading(new Vector2d(48,30))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(Globals.ArmDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelLeftInverse))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterOne,1))


// TODO ========================== DROPPING THE YELLOW AND WHITE PIXEL ==========================================================

                .lineToConstantHeading(new Vector2d(53,26))   // TODO (AT BACKDROP)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.01)
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(0,1))




// TODO ================================================================= TODO FIRST CYCLE START FOR WHITE PIXEL ========================================================
                .lineToConstantHeading(new Vector2d(24, 55))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> Intake.rollOutside(0.9))


                // COMING BACK TO NEUTRAL POS
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.gripSafeOpen())

                .lineToConstantHeading(new Vector2d(-42, 55))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))

                // YELLOW AND WHITE DROPPED
                // TODO TAKING PIXELS FROM STACK
                .lineToLinearHeading(new Pose2d(-59, 40, Math.toRadians(60))) // TODO STACK POSE
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)


                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-45, 53, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Intake.intakeStart(0.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))
                .lineToLinearHeading(new Pose2d(-38, 57, 0))
                .resetConstraints()


                .lineToConstantHeading(new Vector2d(24, 57))

                // TRANSFER SEQUENCE
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.intakeStop())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> robot.flappers.setPosition(Globals.flapperOpen))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.gripSafeOpen())
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Elevator.extendTo(Globals.lifterDown,1))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotatePick())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(Globals.ArmSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.ArmServo(Globals.ArmPick))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0,()->Outtake.gripCloseBoth())


                // todo write command to drop yellow and white pixel5
                .lineToConstantHeading(new Vector2d(48,35))   // TODO (AT BACKDROP)
                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(Globals.ArmDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterOne,1))

                .lineToConstantHeading(new Vector2d(55,32))  // TODO (AT BACKDROP SECOND CALL)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.3)


                .lineToConstantHeading(new Vector2d(50,32))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))


                .lineToConstantHeading(new Vector2d(45,50))        // TODO (PARKING)
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackInit))

                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.gripSafeOpen())
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(-49, 30))  // Dropping Purple Pixel

                // TODO DROPPING THE PURPLE PIXEL
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()->Outtake.setServoShoulder(0.44))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()->Outtake.setServoShoulder(0.14))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->Elevator.extendTo(-50, 0.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.ArmServo(0.3788))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotateInit())

                .lineToConstantHeading(new Vector2d(-38, 30))  // Dropping Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.gripOpenBoth())
                .waitSeconds(0.1)

                // todo Write a command to start intake
                .lineToConstantHeading(new Vector2d(-53, 33))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->Elevator.extendTo(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.3)


                // RE PLUNGING
                .lineToConstantHeading(new Vector2d(-45, 45))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.7))

                .lineToConstantHeading(new Vector2d(-53, 33))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Elevator.extendTo(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

                // todo write trajectories to the backdrop
                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-45, 53, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))
                .lineToLinearHeading(new Pose2d(-35, 57.001, 0))

                .lineToConstantHeading(new Vector2d(24, 57))
                // TRANSFER SEQUENCE


                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.intakeStop())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> robot.flappers.setPosition(Globals.flapperOpen))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.gripSafeOpen())
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Elevator.extendTo(Globals.lifterDown,1))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotatePick())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(Globals.ArmSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.ArmServo(Globals.ArmPick))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->Outtake.gripCloseBoth())


                .lineToConstantHeading(new Vector2d(48,33))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(Globals.ArmDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterOne,1))


// TODO ========================== DROPPING THE YELLOW AND WHITE PIXEL ==========================================================

                .lineToConstantHeading(new Vector2d(53.5,36))   // TODO (AT BACKDROP)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.01)
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterDown,1))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(0,1))




// TODO ================================================================= TODO FIRST CYCLE START FOR WHITE PIXEL ========================================================
                .lineToConstantHeading(new Vector2d(24, 55))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> Intake.rollOutside(0.9))


                // COMING BACK TO NEUTRAL POS
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.gripSafeOpen())


                // YELLOW AND WHITE DROPPED
                .lineToConstantHeading(new Vector2d(-42, 55))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40))

                // TODO TAKING PIXELS FROM STACK
                .lineToLinearHeading(new Pose2d(-59, 40, Math.toRadians(60))) // TODO STACK POSE
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.rollOutside(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackThree))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

                .addTemporalMarker(()-> detect())
                .lineToLinearHeading(new Pose2d(-38, 57, 0))
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Intake.intakeStart(0.4))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))


                .lineToConstantHeading(new Vector2d(24, 57))

                // TRANSFER SEQUENCE
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setServoShoulder(Globals.shoulderSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Intake.intakeStop())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> robot.flappers.setPosition(Globals.flapperOpen))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Outtake.gripSafeOpen())
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Elevator.extendTo(Globals.lifterDown,1))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.rotatePick())
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Outtake.ArmServo(Globals.ArmSafePick))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.ArmServo(Globals.ArmPick))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> Outtake.setServoShoulder(Globals.shoulderPick))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->Outtake.gripCloseBoth())


                // todo write command to drop yellow and white pixel5
                .lineToConstantHeading(new Vector2d(48,35))   // TODO (AT BACKDROP)
                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setServoShoulder(Globals.shoulderDrop))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(Globals.ArmDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Intake.setIntakeServo(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterOne,1))

                .lineToConstantHeading(new Vector2d(53,32))  // TODO (AT BACKDROP SECOND CALL)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Elevator.extendTo(Globals.lifterTwo,1))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(0,1))

                .lineToConstantHeading(new Vector2d(49,32))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.1)
//                .UNSTABLE_addTemporalMarkerOffset(-()-> Intake.setIntakeServo(Globals.stackInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))


                .lineToConstantHeading(new Vector2d(45,50))        // TODO (PARKING)
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackInit))

                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.gripSafeOpen())
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


    public void detect(){
        if(((robot.sensorColor1.red()>=ThresholdColor || robot.sensorColor1.blue()>=ThresholdColor || robot.sensorColor1.green()>=ThresholdColor ) && robot.sensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
                && ((robot.sensorColor2.red()>=ThresholdColor || robot.sensorColor2.blue()>=ThresholdColor || robot.sensorColor2.green()>=ThresholdColor) && robot.sensorColor2.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
            Intake.intakeStart(0.8);
        }
    }

    public void inIt(){
        setServoShoulder(Globals.shoulderSafePick);
        robot.Arm.setPosition(Globals.autoArmInit);
        robot.switchPixel.setPosition(Globals.autoSwitchInit);
        robot.rotate.setPosition(Globals.rotateInit);
        sleep(150);
        robot.flappers.setPosition(Globals.flapperClose);
        sleep(400);
        robot.stackServo.setPosition(Globals.stackInit);
        setServoShoulder(Globals.shoulderInit); //Comment
        sleep(1200);
        robot.leftGrip.setPosition(Globals.autoleftGripInit);
        robot.rightGrip.setPosition(Globals.autorightGripInit);
    }

}
