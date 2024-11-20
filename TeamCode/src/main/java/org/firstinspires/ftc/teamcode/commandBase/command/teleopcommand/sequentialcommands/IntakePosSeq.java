package org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.RotateCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.SwitchPixelCommand;

public class IntakePosSeq extends SequentialCommandGroup {

    public IntakePosSeq(OutakeSubsystem Outake, IntakeSubsystem Intake,ElevatorSubsytem Elevator) {
        super(
                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_OPEN),
                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INIT),
                new FlapperCommand(Intake, IntakeSubsystem.FlappersState.FLAPPER_CLOSE),
                new WaitCommand(100),
                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HOME,0),
                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INIT),
                new ArmCommand(Outake, OutakeSubsystem.ArmState.INIT),
//                new WaitCommand(250),
                new ShoulderCommand(Outake, OutakeSubsystem.ShoulderState.INIT),
                new RotateCommand(Outake, OutakeSubsystem.RotateState.INIT),
                new SwitchPixelCommand(Outake, OutakeSubsystem.SwitchPixelState.SWITCH_INIT),
                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_SAFE)
        );
    }


}
