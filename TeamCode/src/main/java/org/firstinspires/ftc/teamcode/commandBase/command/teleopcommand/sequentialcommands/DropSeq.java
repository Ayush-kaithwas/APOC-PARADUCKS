package org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.RotateCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.SwitchPixelCommand;

public class DropSeq extends SequentialCommandGroup {

    public DropSeq(IntakeSubsystem intake, OutakeSubsystem outake) {
        super(
                new IntakeMotorCommand(intake, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new IntakeServoCommand(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
                new WaitCommand(100),
                new ArmCommand(outake,OutakeSubsystem.ArmState.SAFEDROP),
                new RotateCommand(outake, OutakeSubsystem.RotateState.PREDROP),
//                new WaitCommand(100),
                new ArmCommand(outake,OutakeSubsystem.ArmState.PREDROP),
//                new WaitCommand(150),
                new ShoulderCommand(outake, OutakeSubsystem.ShoulderState.DROP),
                new SwitchPixelCommand(outake, OutakeSubsystem.SwitchPixelState.SWITCH_DROP),
                new RotateCommand(outake,OutakeSubsystem.RotateState.PLACE), // 0.5166
                new ArmCommand(outake, OutakeSubsystem.ArmState.DROP), // 0.3905
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_CLOSE)


        );
    }

    public DropSeq(IntakeSubsystem intake, OutakeSubsystem outake, ElevatorSubsytem elevator, ElevatorSubsytem.ElevateState state, OutakeSubsystem.SwitchPixelState switchState) {
        super(
                new IntakeMotorCommand(intake, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new IntakeServoCommand(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
                new WaitCommand(100),
                new ArmCommand(outake,OutakeSubsystem.ArmState.SAFEDROP),
                new RotateCommand(outake, OutakeSubsystem.RotateState.PREDROP),
//                new WaitCommand(100),
                new ArmCommand(outake,OutakeSubsystem.ArmState.PREDROP),
//                new WaitCommand(150),
                new ShoulderCommand(outake, OutakeSubsystem.ShoulderState.DROP),
                new SwitchPixelCommand(outake, switchState),
                new ElevatorCommand(elevator, state, 0),
                new RotateCommand(outake,OutakeSubsystem.RotateState.PLACE), // 0.5166
                new ArmCommand(outake, OutakeSubsystem.ArmState.DROP), // 0.3905
                new IntakeServoCommand(intake, IntakeSubsystem.IntakeServoState.INIT),
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_CLOSE)


        );

    }
}
/*
       new IntakeMotorCommand(intake, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new IntakeServoCommand(intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
                new WaitCommand(100),
                new ArmCommand(outake,OutakeSubsystem.ArmState.SAFEDROP),
                new RotateCommand(outake, OutakeSubsystem.RotateState.PREDROP),
//                new WaitCommand(100),
                new ArmCommand(outake,OutakeSubsystem.ArmState.PREDROP),
//                new WaitCommand(150),
                new ShoulderCommand(outake, OutakeSubsystem.ShoulderState.DROP),
                new SwitchPixelCommand(outake, switchState),
                new ElevatorCommand(elevator, state, 0),
                new RotateCommand(outake,OutakeSubsystem.RotateState.PLACE), // 0.5166
                new ArmCommand(outake, OutakeSubsystem.ArmState.DROP), // 0.3905
                new IntakeServoCommand(intake, IntakeSubsystem.IntakeServoState.INIT),
                new FlapperCommand(intake, IntakeSubsystem.FlappersState.FLAPPER_CLOSE)
 */

