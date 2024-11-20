package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class IntakeMotorCommand extends InstantCommand {
    public IntakeMotorCommand(IntakeSubsystem intake, IntakeSubsystem.RollerIntakeState state) {
        super(
                ()->intake.updateState(state)
        );
    }
}
