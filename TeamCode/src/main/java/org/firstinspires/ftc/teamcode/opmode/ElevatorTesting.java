package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class ElevatorTesting extends CommandOpMode {

    public IntakeSubsystem intake = null;
    public OutakeSubsystem outtake = null;
    public ElevatorSubsytem elevator = null;
    private final RobotHardware robot = RobotHardware.getInstance();   //Robot instance

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);
        elevator = new ElevatorSubsytem(robot);

    }

    @Override
    public  void run()
    {
        super.run();


        if(gamepad1.dpad_up)
        {
            inc(0.7);
        }
        else if(gamepad1.dpad_down){
            dec(0.7);
        }
        telemetry.addData("Left Elevator POS",robot.leftElevator.getCurrentPosition());
        telemetry.addData("Right Elevator Pos",robot.rightElevator.getCurrentPosition());
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
