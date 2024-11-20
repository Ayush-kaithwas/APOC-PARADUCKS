package org.firstinspires.ftc.teamcode.commandBase.command.instantcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;

import java.security.cert.Extension;

public class ExtensionCommand extends InstantCommand {
    public ExtensionCommand(ExtensionSubsystem Extension, ExtensionSubsystem.IntakeExtensionState state) {
        super(
                ()-> Extension.updateState(state)
        );
    }
}
