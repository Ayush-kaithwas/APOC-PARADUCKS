package org.firstinspires.ftc.teamcode.Auto.Red;

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
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Location;
import org.firstinspires.ftc.teamcode.hardware.PropPipeline;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
@Config
public class RedClose60Unstable extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {
        drive=new SampleMecanumDrive(hardwareMap);
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;

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




        Pose2d startPose = new Pose2d(14, -62, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(23, -22.2))//Center /
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})//-65Best //time can be saved here
                //todo purple dropped

                .lineToConstantHeading(new Vector2d(44.5,-34.5))//can remove this
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
                .lineToConstantHeading(new Vector2d(48.8,-34.5))
                .waitSeconds(0.01)
                .addTemporalMarker(()->{Outtake.gripOpenBoth();})
                .waitSeconds(0.15)
                //todo yellow dropped YELLOW   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

                //Todo first cycle-----------------------------------------------------------------------------------------------------------------------------------------
                //todo write trajectories shifting from backdrop towards wall   -----------------------

                .lineToLinearHeading(new Pose2d(24,-57, Math.toRadians(0) ))


                //Sequence for robot init        and     setting stack height //todo stack height (autostackInit)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Outtake.setSwitchPixel(Globals.switchPixelInit);Outtake.ArmServo(Globals.ArmInit);})
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{ Outtake.shoulderInit();Outtake.rotateInit();robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);})
                .UNSTABLE_addTemporalMarkerOffset(0,()->Intake.rollOutside(0.7))
                .waitSeconds(0.01)


                //todo write trajectories to towards JUST BEFOREstack--------------------------------------------

                .lineToConstantHeading(new Vector2d(-47, -55))//45
                .addTemporalMarker(()->Extension.Extension(50,0.9))

                .lineToLinearHeading(new Pose2d(-61, -45.8, Math.toRadians(135))) //180-45//new Pose2d(-62, 40, Math.toRadians(60)

                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFour))
//                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.5)
//                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackThree))
//                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-53, -45.8))


//                .lineToLinearHeading(new Pose2d(-61, 45.8001, Math.toRadians(55)))//new Pose2d(-62, 40, Math.toRadians(60)  Bets

                .lineToLinearHeading(new Pose2d(-61, -45.8001, Math.toRadians(135)))//new Pose2d(-62, 40, Math.toRadians(60)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {Extension.Extension(0,0.9);Intake.setIntakeServo(Globals.stackFive);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

//                .lineToConstantHeading(new Vector2d(-37, 55))//45///Bestt
                .lineToLinearHeading(new Pose2d(-37,-57,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.intakeStart(0.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))


                .lineToLinearHeading(new Pose2d(40,-57,Math.toRadians(0)))   // TODO (AT BACKDROP)
//
//                // TRANSFER SEQUENCE
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


                // todo write command to drop yellow

                .lineToLinearHeading(new Pose2d(47,-48.55,Math.toRadians(215)))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> {Outtake.setServoShoulder(0.2177);Elevator.extendTo(Globals.lifterFive,1);})
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(0.4466))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())// change this
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> {;Outtake.setRotate(0.7355);Outtake.setSwitchPixel(0.2966);})
                //2 pxl dropped
                .lineToLinearHeading(new Pose2d(47.5001,-48.55,Math.toRadians(215)))   // TODO (AT BACKDROP)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.15)
                .addTemporalMarker(()->{Outtake.setSwitchPixel(Globals.switchPixelInit);})
                .addTemporalMarker(()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
                .addTemporalMarker(()->Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.01)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30, -31))//Left30.3, 31E
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})//-65Best //time can be saved here
                //todo purple dropped

                .lineToConstantHeading(new Vector2d(44.5,-39))//can remove this
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
                .lineToConstantHeading(new Vector2d(48.8,-39))//48.5
                .waitSeconds(0.01)
                .addTemporalMarker(()->{Outtake.gripOpenBoth(); robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);})
                .waitSeconds(0.15)
                //todo yellow dropped YELLOW   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

                //Todo first cycle-----------------------------------------------------------------------------------------------------------------------------------------

                //Todo first cycle-----------------------------------------------------------------------------------------------------------------------------------------
                //todo write trajectories shifting from backdrop towards wall   -----------------------

                .lineToLinearHeading(new Pose2d(24,-57, Math.toRadians(0) ))


                //Sequence for robot init        and     setting stack height //todo stack height (autostackInit)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Outtake.setSwitchPixel(Globals.switchPixelInit);Outtake.ArmServo(Globals.ArmInit);})
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{ Outtake.shoulderInit();Outtake.rotateInit();robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);})
                .UNSTABLE_addTemporalMarkerOffset(0,()->Intake.rollOutside(0.7))
                .waitSeconds(0.01)


                //todo write trajectories to towards JUST BEFOREstack--------------------------------------------

                .lineToConstantHeading(new Vector2d(-47, -55))//45
                .addTemporalMarker(()->Extension.Extension(50,0.9))

                .lineToLinearHeading(new Pose2d(-61, -45.8, Math.toRadians(235)))//new Pose2d(-62, 40, Math.toRadians(60)

                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFour))
//                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.5)
//                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackThree))
//                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-53, -45.8))


//                .lineToLinearHeading(new Pose2d(-61, 45.8001, Math.toRadians(55)))//new Pose2d(-62, 40, Math.toRadians(60)  Bets

                .lineToLinearHeading(new Pose2d(-61, -45.8001, Math.toRadians(135)))//new Pose2d(-62, 40, Math.toRadians(60)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {Extension.Extension(0,0.9);Intake.setIntakeServo(Globals.stackFive);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

//                .lineToConstantHeading(new Vector2d(-37, 55))//45///Bestt
                .lineToLinearHeading(new Pose2d(-37,-57,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.intakeStart(0.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))


                .lineToLinearHeading(new Pose2d(40,-57,Math.toRadians(0)))   // TODO (AT BACKDROP)
//
//                // TRANSFER SEQUENCE
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


                // todo write command to drop yellow

                .lineToLinearHeading(new Pose2d(47,-48.55,Math.toRadians(215)))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> {Outtake.setServoShoulder(0.2177);Elevator.extendTo(Globals.lifterFive,1);})
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(0.4466))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())// change this
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> {;Outtake.setRotate(0.7355);Outtake.setSwitchPixel(0.2966);})
                //2 pxl dropped
                .lineToLinearHeading(new Pose2d(47.5001,-48.55,Math.toRadians(215)))   // TODO (AT BACKDROP)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.15)
                .addTemporalMarker(()->{Outtake.setSwitchPixel(Globals.switchPixelInit);})
                .addTemporalMarker(()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
                .addTemporalMarker(()->Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.01)
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                //todo purple drop pose  ----------------------------------------------------------
                .lineToConstantHeading(new Vector2d(22, -32))
                .lineToConstantHeading(new Vector2d(10, -32))

                //todo code to drop purple
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})

                //todo purple dropped   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


                //todo backdrop pose for YELLOW-----------------------------------------------------

                .lineToConstantHeading(new Vector2d(48.8,-27.8))
                .waitSeconds(0.01)
                .lineToConstantHeading(new Vector2d(48.80001,-27.8))  //backdrop re-lock
                .addTemporalMarker(()->{Outtake.gripOpenBoth();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->setServoShoulder(Globals.shoulderYellowSafe))

                //todo yellow dropped YELLOW   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

                //Todo first cycle-----------------------------------------------------------------------------------------------------------------------------------------
                //todo write trajectories shifting from backdrop towards wall   -----------------------

                .lineToLinearHeading(new Pose2d(24,-57, Math.toRadians(0) ))


                //Sequence for robot init        and     setting stack height //todo stack height (autostackInit)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Outtake.setSwitchPixel(Globals.switchPixelInit);Outtake.ArmServo(Globals.ArmInit);})
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{ Outtake.shoulderInit();Outtake.rotateInit();robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);})
                .UNSTABLE_addTemporalMarkerOffset(0,()->Intake.rollOutside(0.7))
                .waitSeconds(0.01)


                //todo write trajectories to towards JUST BEFOREstack--------------------------------------------

                .lineToConstantHeading(new Vector2d(-47, -55))//45
                .addTemporalMarker(()->Extension.Extension(50,0.9))

                .lineToLinearHeading(new Pose2d(-61, -45.8, Math.toRadians(125)))//new Pose2d(-62, 40, Math.toRadians(60)

                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFour))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.setIntakeServo(Globals.stackFive))
                .waitSeconds(0.5)

                .lineToConstantHeading(new Vector2d(-53, -45.8))



                .lineToLinearHeading(new Pose2d(-61, -45.8001, Math.toRadians(135)))//new Pose2d(-62, 40, Math.toRadians(60)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {Extension.Extension(0,0.9);Intake.setIntakeServo(Globals.stackFive);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Intake.setIntakeServo(Globals.stackDown))
                .waitSeconds(0.3)

//                .lineToConstantHeading(new Vector2d(-37, 55))//45///Bestt
                .lineToLinearHeading(new Pose2d(-37,-57,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.intakeStart(0.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()-> Intake.setIntakeServo(Globals.stackDown))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Extension.Extension(50,1))


                .lineToLinearHeading(new Pose2d(40,-57,Math.toRadians(0)))   // TODO (AT BACKDROP)
//
//                // TRANSFER SEQUENCE
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


                // todo write command to drop yellow

                .lineToLinearHeading(new Pose2d(47,-48.55,Math.toRadians(215)))   // TODO (AT BACKDROP)

                // DROPPING SEQ
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(0.875))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.rotatePreDrop())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> {Outtake.setServoShoulder(0.2177);Elevator.extendTo(Globals.lifterFive,1);})
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> Outtake.ArmServo(0.4466))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> Outtake.setSwitchPixel(Globals.switchPixelDrop))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->Outtake.rotateDrop())// change this
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> robot.flappers.setPosition(Globals.flapperClose))
                .addTemporalMarker(()-> {;Outtake.setRotate(0.7355);Outtake.setSwitchPixel(0.2966);})
                //2 pxl dropped
                .lineToLinearHeading(new Pose2d(47.5001,-48.55,Math.toRadians(215)))   // TODO (AT BACKDROP)
                .addTemporalMarker(()-> Outtake.gripOpenBoth())
                .waitSeconds(0.15)
                .addTemporalMarker(()->{Outtake.setSwitchPixel(Globals.switchPixelInit);})
                .addTemporalMarker(()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
                .addTemporalMarker(()->Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.01)
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
    public void intakeMotorCounts(int counts,double pow){
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMotor.setTargetPosition(counts);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeMotor.setPower(pow);
    }

}
