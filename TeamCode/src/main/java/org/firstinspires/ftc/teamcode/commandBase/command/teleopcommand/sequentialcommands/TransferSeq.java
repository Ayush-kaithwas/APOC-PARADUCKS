package org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.RotateCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.SwitchPixelCommand;

public class TransferSeq extends SequentialCommandGroup {


    public TransferSeq(IntakeSubsystem Intake, OutakeSubsystem Outake, ElevatorSubsytem elevate) {
        super(

                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.SAFEPICK),
                new SwitchPixelCommand(Outake, OutakeSubsystem.SwitchPixelState.SWITCH_INIT),
//                new WaitCommand(250),
                new WaitCommand(100),
                new IntakeMotorCommand(Intake, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new FlapperCommand(Intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
//                new WaitCommand(200),
                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_SAFE),
                new ElevatorCommand(elevate, ElevatorSubsytem.ElevateState.HOME, 0),
                new RotateCommand(Outake, OutakeSubsystem.RotateState.PICK),
                new ArmCommand(Outake,OutakeSubsystem.ArmState.SAFEPICK),
                new WaitCommand(250),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.INIT),
//                new WaitCommand(300),
                new WaitCommand(100).andThen(new ArmCommand(Outake,OutakeSubsystem.ArmState.PICK)),
                new WaitCommand(100),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.PICK),
                new WaitCommand(250).andThen(new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_CLOSE))
        );
    }public TransferSeq(IntakeSubsystem Intake, OutakeSubsystem Outake, ElevatorSubsytem elevate, ExtensionSubsystem extension) {
        super(

                new ExtensionCommand(extension, ExtensionSubsystem.IntakeExtensionState.PULL_IN),
                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.SAFEPICK),
                new SwitchPixelCommand(Outake, OutakeSubsystem.SwitchPixelState.SWITCH_INIT),
//                new WaitCommand(250),
                new WaitCommand(100),
                new IntakeMotorCommand(Intake, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new FlapperCommand(Intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
//                new WaitCommand(200),
                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_SAFE),
                new ElevatorCommand(elevate, ElevatorSubsytem.ElevateState.HOME, 0),
                new RotateCommand(Outake, OutakeSubsystem.RotateState.PICK),
                new ArmCommand(Outake,OutakeSubsystem.ArmState.SAFEPICK),
                new WaitCommand(150),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.INIT),
//                new WaitCommand(300),
                new WaitCommand(100).andThen(new ArmCommand(Outake,OutakeSubsystem.ArmState.PICK)),
                new WaitCommand(100),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.PICK),
                new WaitCommand(250).andThen(new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_CLOSE)),
                new ExtensionCommand(extension, ExtensionSubsystem.IntakeExtensionState.INIT)
        );
    }
}
