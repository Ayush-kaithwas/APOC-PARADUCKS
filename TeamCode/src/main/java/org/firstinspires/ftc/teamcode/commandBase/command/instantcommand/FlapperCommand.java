package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class FlapperCommand extends InstantCommand {

    public FlapperCommand(IntakeSubsystem intake, IntakeSubsystem.FlappersState state) {
        super(
                ()->intake.updateState(state)
        );
    }
}
