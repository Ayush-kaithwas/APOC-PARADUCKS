package org.firstinspires.ftc.teamcode.Auto.Blue;

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
public class BlueClose50Unstable extends LinearOpMode {

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

    //Poses
    Vector2d purpleDropCenter=new Vector2d(46.2,36);



    @Override
    public void runOpMode() throws InterruptedException {
        drive=new SampleMecanumDrive(hardwareMap);
        Globals.ALLIANCE = Location.BLUE;
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
            catch (Exception e){
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
            robot.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("motorCount",robot.intakeMotor.getCurrentPosition());
            telemetry.update();
            robot.ratchet.setPosition(Globals.RachetOpen);
        }
        portal.close();



//        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(0)); // Working good for 60 Inner Tab
        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(0)); // Working good for 60 Inner Tab
        drive.setPoseEstimate(startPose);



//        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(48,29.501))
////                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})
//                .UNSTABLE_addTemporalMarkerOffset(-2,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->Extension(-1600,1))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
//                Outtake.gripOpenBoth();})
//                .waitSeconds(0.15)
//                .lineToLinearHeading(new Pose2d(46,29.5001, Math.toRadians(0) ))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(-70,0.9);
//                    Outtake.gripSafeOpen();})
//                .addTemporalMarker(()->Extension(0,1))
//                .lineToLinearHeading(new Pose2d(24,57, Math.toRadians(0) ))
//                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->Outtake.setSwitchPixel(Globals.switchPixelInit))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
//                .waitSeconds(4)
//                .build();
//
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22, 32))  //right//19 f
                .lineToConstantHeading(new Vector2d(9.5, 32))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})//-65Best //time can be saved here
                //todo purple dropped

                .lineToConstantHeading(new Vector2d(44.5,28.5))//can remove this
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
                .lineToConstantHeading(new Vector2d(48.8,28.5))//48.5
                .waitSeconds(0.01)
                .addTemporalMarker(()->{Outtake.gripOpenBoth();})
                .waitSeconds(0.15)
                // todo After dropping yellow
                // todo After dropping yellow (First Cycle)

                .lineToLinearHeading(new Pose2d(43,28.5001, Math.toRadians(0) ))//pixel safety can remove this
                .lineToLinearHeading(new Pose2d(43,57, Math.toRadians(0) ))
                //ReadyToInit
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
                .waitSeconds(0.01)


                //Todo faster

//                        .lineToConstantHeading(new Vector2d(24, 32))  //right
//                        .lineToConstantHeading(new Vector2d(8, 32))
//                        .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
//                        .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})//-65Best //time can be saved here
//                        .lineToConstantHeading(new Vector2d(48,29.501))
//                        .waitSeconds(0.01)
//                        .addTemporalMarker(()->{Outtake.gripOpenBoth();})
//                        .waitSeconds(0.15)
//                        .lineToLinearHeading(new Pose2d(44,29.5001, Math.toRadians(0) ))//INSTEAD OF THIS MOVE ARM
//                        .lineToLinearHeading(new Pose2d(24,57, Math.toRadians(0) ))
//                        .UNSTABLE_addTemporalMarkerOffset(-0.3,()->Outtake.setSwitchPixel(Globals.switchPixelInit))
//                        .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
//                        .waitSeconds(0.01)

                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30, 31))//Left30.3, 31E
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})//-65Best //time can be saved here
                //todo purple dropped

                .lineToConstantHeading(new Vector2d(44.5,39))//can remove this
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
                .lineToConstantHeading(new Vector2d(48.8,39))//48.5
                .waitSeconds(0.01)
                .addTemporalMarker(()->{Outtake.gripOpenBoth();})
                .waitSeconds(0.15)
                // todo After dropping yellow
                // todo After dropping yellow (First Cycle)
                .lineToLinearHeading(new Pose2d(43,39, Math.toRadians(0) ))//pixel safety can remove this
                .lineToLinearHeading(new Pose2d(43,57, Math.toRadians(0) ))
                //ReadyToInit
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->Outtake.setSwitchPixel(Globals.switchPixelInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{Outtake.shoulderInit();Outtake.rotateInit();Intake.stackInit();})
                .waitSeconds(0.01)
                .build();



        TrajectorySequence center  = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(23, 22.2))//Center /
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{intakeMotorCounts(70,0.9);})//-65Best //time can be saved here
                //todo purple dropped

                .lineToConstantHeading(new Vector2d(44.5,34.5))//can remove this
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{setServoShoulder(Globals.shoulderDrop);Outtake.ArmServo(Globals.autoArmYellow);})//-0.8b
                .lineToConstantHeading(new Vector2d(48.8,34.5))
                .waitSeconds(0.01)
                .addTemporalMarker(()->{Outtake.gripOpenBoth();})
                .waitSeconds(0.15)
                // todo After dropping yellow
                // todo After dropping yellow (First Cycle)
                .lineToLinearHeading(new Pose2d(43,34.5, Math.toRadians(0) ))//pixel safety can remove this
                .lineToLinearHeading(new Pose2d(43,57, Math.toRadians(0) ))
                //ReadyToInit
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Intake.setIntakeServo(Globals.stackInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.6,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> Elevator.extendTo(Globals.lifterDown,1))
                .waitSeconds(0.2)
//                .UNSTABLE_addTemporalMarkerOffset(-()-> Intake.setIntakeServo(Globals.stackInit))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()-> Outtake.ArmServo(Globals.ArmInit))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> Outtake.setServoShoulder(Globals.shoulderInit))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()-> Outtake.rotateInit())
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> Outtake.setSwitchPixel(Globals.switchPixelInit))
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
        robot.leftGrip.setPosition(0.932);
        robot.rightGrip.setPosition(0.005);
    }
}

