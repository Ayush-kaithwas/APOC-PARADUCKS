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

public class TransferSeq extends SequentialCommandGroup {


    public TransferSeq(IntakeSubsystem Intake, OutakeSubsystem Outake, ElevatorSubsytem elevate) {
        super(

                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.SAFEPICK),
                new SwitchPixelCommand(Outake, OutakeSubsystem.SwitchPixelState.SWITCH_INIT),
                new WaitCommand(250),
                new FlapperCommand(Intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
                new WaitCommand(200),
                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_SAFE),
                new ElevatorCommand(elevate, ElevatorSubsytem.ElevateState.HOME, 0),
                new RotateCommand(Outake, OutakeSubsystem.RotateState.PICK),
                new WaitCommand(250),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.INIT),
                new ArmCommand(Outake,OutakeSubsystem.ArmState.SAFEPICK),
                new WaitCommand(300),
                new WaitCommand(150).andThen(new ArmCommand(Outake,OutakeSubsystem.ArmState.PICK)),
                new WaitCommand(150),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.PICK),
                new WaitCommand(1250).andThen(new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_CLOSE))


//                new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
//                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.SAFEPICK),
//                new SwitchPixelCommand(Outake, OutakeSubsystem.SwitchPixelState.SWITCH_INIT),
//                new WaitCommand(250),
//                new FlapperCommand(Intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
//                new WaitCommand(200),
//                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_SAFE),
//                new ElevatorCommand(elevate, ElevatorSubsytem.ElevateState.HOME, 0),
//                new RotateCommand(Outake, OutakeSubsystem.RotateState.PICK),
//                new ArmCommand(Outake,OutakeSubsystem.ArmState.SAFEPICK),
//                new WaitCommand(300),
//                new WaitCommand(150).andThen(new ArmCommand(Outake,OutakeSubsystem.ArmState.PICK)),
//                new WaitCommand(150),
//                new WaitCommand(150).andThen(new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.SAFEPICK1)),
//                new WaitCommand(150),
//                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.PICK),
//                new WaitCommand(1250).andThen(new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_CLOSE))

        );
    }
}

//////////////// TODO PICKING POSITIONS
/*

OG WOrking
     new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.SAFEPICK),
                new SwitchPixelCommand(Outake, OutakeSubsystem.SwitchPixelState.SWITCH_INIT),
                new WaitCommand(250),
                new FlapperCommand(Intake, IntakeSubsystem.FlappersState.FLAPPER_OPEN),
                new WaitCommand(200),
                new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_SAFE),
                new ElevatorCommand(elevate, ElevatorSubsytem.ElevateState.HOME, 0),
                new RotateCommand(Outake, OutakeSubsystem.RotateState.PICK),
                new ArmCommand(Outake,OutakeSubsystem.ArmState.SAFEPICK),
                new WaitCommand(300),
                new WaitCommand(150).andThen(new ArmCommand(Outake,OutakeSubsystem.ArmState.PICK)),
                new WaitCommand(250),
                new ShoulderCommand(Outake,OutakeSubsystem.ShoulderState.PICK),
                new WaitCommand(1250).andThen(new GripperCommand(Outake, OutakeSubsystem.GripperState.GRIP_CLOSE))


 * Rotate = 0.5
 * Flapper = 0.54 // Change
 * Left Grip = 0.705
 * Right Grip = 0.5
 * ARM = 0.706
 * Switch Pixel = 0.4777
 * Right S  = 0.0116
 * Left S = 0.9877
 * */

// GRAB THE PIXEL
//    Right GRIP  = 0.445
// Left Grip = 0.879


////////////////// TODO DROP PIXEL
/*
 * Rotate = 0.5
 * Flapper = 0.54 // Change
 * Left Grip = 0.705 // Change
 * Right Grip = 0.5 // Change
 * ARM = 0.30166
 * Switch Pixel = 0.480
 * Right S  = 0.5977
 * Left S = 0.40166
 * */