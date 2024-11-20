package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;

public class GripperCommand extends InstantCommand {
    public GripperCommand(OutakeSubsystem outake, OutakeSubsystem.GripperState state) {
        super(
                ()->outake.updateState(state)
        );
    }
}
