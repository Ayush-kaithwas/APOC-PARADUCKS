package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;

public class RotateCommand extends InstantCommand {
    public RotateCommand(OutakeSubsystem outake, OutakeSubsystem.RotateState state) {
        super(
                ()->outake.updateState(state)
        );
    }
}
