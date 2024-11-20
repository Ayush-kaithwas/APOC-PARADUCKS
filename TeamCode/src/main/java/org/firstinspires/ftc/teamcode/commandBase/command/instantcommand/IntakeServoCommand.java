package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class IntakeServoCommand extends InstantCommand {
    public IntakeServoCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeServoState state) {
        super(
                ()->intake.updateState(state)
        );
    }
}
