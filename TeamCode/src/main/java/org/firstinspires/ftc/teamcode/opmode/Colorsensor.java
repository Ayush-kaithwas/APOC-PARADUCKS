package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.DropSeq;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.IntakePixel;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.TransferSeq;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Config


@TeleOp(name = "Color Sensor", group = "Sensor")
public class Colorsensor extends CommandOpMode {


    private final RobotHardware robot = RobotHardware.getInstance();   //Robot instance
    public int ThresholdColor=1000;
    public int ThresholdDistance=15;

    public IntakeSubsystem intake = null;
    public OutakeSubsystem outtake = null;
    public ElevatorSubsytem elevator = null;

    public ExtensionSubsystem extension = null;


    SampleMecanumDrive drive = null;

    boolean PixelIntake = TRUE;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap,telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeSubsystem(robot);
        outtake = new OutakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);
        extension = new ExtensionSubsystem(robot);



        robot.flappers.setPosition(Globals.flapperClose);
        sleep(200);
        robot.stackServo.setPosition(Globals.stackInit);
        sleep(150);
        robot.Arm.setPosition(Globals.ArmInit);
        setServoShoulder(Globals.shoulderInit);
        robot.rotate.setPosition(Globals.rotateInit);
        robot.switchPixel.setPosition(Globals.switchPixelInit);

    }

    @Override
    public  void run()
    {
        super.run();
        if(((robot.sensorColor1.red()>=ThresholdColor || robot.sensorColor1.blue()>=ThresholdColor || robot.sensorColor1.green()>=ThresholdColor ) && robot.sensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
                && ((robot.sensorColor2.red()>=ThresholdColor || robot.sensorColor2.blue()>=ThresholdColor || robot.sensorColor2.green()>=ThresholdColor) && robot.sensorColor2.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
            schedule( new TransferSeq(intake, outtake, elevator));
        }
        if(gamepad1.b)
        {
            schedule(  new IntakePixel(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_ON)); // Intaking the Pixel From ground
        }
        else if (gamepad1.a) {
            schedule(  new IntakePixel(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_OFF));
        }
        else if(gamepad1.left_bumper){
            schedule(new DropSeq(intake,outtake));
        }
        else if(gamepad1.x){
            robot.flappers.setPosition(Globals.flapperClose);
        }
        else if(gamepad1.y){
            schedule(new GripperCommand(outtake, OutakeSubsystem.GripperState.GRIP_OPEN));
        }
        else if(gamepad1.right_bumper)
        {
            robot.ratchet.setPosition(0.2);
        }
        else if(gamepad1.left_trigger>0){
            robot.ratchet.setPosition(1);
        }

    }

    public void setServoShoulder(double leftPos){    //Todo add servo offset if needed.
        double rightPos=1-leftPos;
        robot.leftShoulder.setPosition(leftPos);
        robot.rightShoulder.setPosition(rightPos);
    }



}
