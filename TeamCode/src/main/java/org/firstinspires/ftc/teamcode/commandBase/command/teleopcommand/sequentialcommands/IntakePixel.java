package org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeServoCommand;

public class IntakePixel extends SequentialCommandGroup {
    public IntakePixel(IntakeSubsystem Intake, IntakeSubsystem.IntakeServoState stackState , IntakeSubsystem.RollerIntakeState rollerState) {
        super(
                new IntakeServoCommand(Intake,stackState), //  Intake Motor Start
                new IntakeMotorCommand(Intake, rollerState) // Stack Servo down
        );
    }
}
