package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;

public class ElevatorCommand extends InstantCommand {
    public ElevatorCommand(ElevatorSubsytem elevator, ElevatorSubsytem.ElevateState state, int dropheight) {
        super(
                ()->elevator.updateState(state,dropheight)
        );
    }
}
