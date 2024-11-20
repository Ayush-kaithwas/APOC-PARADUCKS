package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;

public class ArmCommand extends InstantCommand {

    public ArmCommand(OutakeSubsystem Outake, OutakeSubsystem.ArmState state) {
        super(
                ()->Outake.updateState(state)
        );
    }
}
