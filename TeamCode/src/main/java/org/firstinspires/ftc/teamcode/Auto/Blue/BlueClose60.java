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
public class BlueClose60 extends LinearOpMode {

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
            telemetry.update();
        }
        portal.close();




        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(180)); // original
        drive.setPoseEstimate(startPose);


//

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)


                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .build();



        waitForStart();

        if(marker==Location.LEFT){
            drive.followTrajectorySequence(left);
        }
        else if (marker==Location.RIGHT)
        {
            drive.followTrajectorySequence(right);
        }
        else
        {
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

}
