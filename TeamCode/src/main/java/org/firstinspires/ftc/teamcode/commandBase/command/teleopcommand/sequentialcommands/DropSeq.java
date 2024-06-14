package org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.RotateCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.SwitchPixelCommand;

public class DropSeq extends SequentialCommandGroup {

    public DropSeq(IntakeSubsystem intake, OutakeSubsystem outake) {
        super(
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
                new WaitCommand(250),
                new ArmCommand(outake,OutakeSubsystem.ArmState.SAFEDROP),
                new RotateCommand(outake, OutakeSubsystem.RotateState.PREDROP),
                new WaitCommand(150),
                new ArmCommand(outake,OutakeSubsystem.ArmState.PREDROP),
                new WaitCommand(250),
                new ShoulderCommand(outake, OutakeSubsystem.ShoulderState.DROP), // shoulderPick= 0.9877;
                new WaitCommand(500),
                new RotateCommand(outake,OutakeSubsystem.RotateState.PLACE),
                new ArmCommand(outake, OutakeSubsystem.ArmState.DROP),
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_CLOSE)


        );
    }
}


