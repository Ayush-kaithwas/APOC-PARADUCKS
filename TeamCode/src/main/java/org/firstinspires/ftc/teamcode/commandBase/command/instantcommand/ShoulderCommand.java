package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;

public class ShoulderCommand extends InstantCommand {
    public ShoulderCommand(OutakeSubsystem outake, OutakeSubsystem.ShoulderState state) {
        super(
                ()->outake.updateState(state)
        );
    }
}
