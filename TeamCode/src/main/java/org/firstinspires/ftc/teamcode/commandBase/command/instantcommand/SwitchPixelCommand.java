package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;

public class SwitchPixelCommand extends InstantCommand {
    public SwitchPixelCommand(OutakeSubsystem outake, OutakeSubsystem.SwitchPixelState state) {
     super(
             ()->outake.updateState(state)
     );
    }


}
