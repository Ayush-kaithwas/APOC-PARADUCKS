package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.Subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.FlapperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.GripperCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.RotateCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.instantcommand.SwitchPixelCommand;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.DropSeq;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.IntakePixel;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.IntakePosSeq;
import org.firstinspires.ftc.teamcode.commandBase.command.teleopcommand.sequentialcommands.TransferSeq;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.security.cert.Extension;

import dalvik.system.DelegateLastClassLoader;

@TeleOp
@Config


public class ParasTeleop extends CommandOpMode {

    public IntakeSubsystem Intake = null;
    public OutakeSubsystem Outtake = null;
    public ElevatorSubsytem Elevator = null;

    public ExtensionSubsystem Extension = null;

    public static int ElevatorPOs;
    public int counter =0;
    public int[] ElevatorStates={1,2,3,4,5,6,7,8,9};  //Use this for counter
    public int ElevatorCounter=0;
    public static boolean pixelDrop = false;
    SampleMecanumDrive drive = null;
    private final RobotHardware robot = RobotHardware.getInstance();   //Robot instance

    boolean PixelIntake = false;
    public static int dropHeightOne = 0;
    public static int ThresholdDistance=15;
    public static int ThresholdColor=700;
    public static boolean extensionFlag = true;
    public GamepadEx gamepadEx;
    public GamepadEx gamepadEx2;
    public boolean extendit = false;
    public boolean isIntake = false;
    public boolean isDrop = false;


    public static double head = 0.5;
    public static double straight = 0.8;
    public static double strafe = 0.3;
    public static int ElevatorPos;
    public static int sliderPos;

    public static boolean isElevate= false;
    public static boolean shiftPixel= false;


    public OutakeSubsystem.SwitchPixelState storeState= OutakeSubsystem.SwitchPixelState.SWITCH_HORIZONTAL;

    public boolean beamFlag = true;



 public ElapsedTime timer = new ElapsedTime();
    private boolean inversePixelFlag = false;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        Intake = new IntakeSubsystem(robot);
        Outtake = new OutakeSubsystem(robot);
        Elevator = new ElevatorSubsytem(robot);
        Extension = new ExtensionSubsystem(robot);
        gamepadEx=new GamepadEx(gamepad1);

        gamepadEx2 = new GamepadEx(gamepad2);
        robot.beam1.setMode(DigitalChannel.Mode.INPUT);
        robot.beam2.setMode(DigitalChannel.Mode.INPUT);


        //// TODO ================================================== INIT ================================================
        robot.stackServo.setPosition(Globals.stackInit);
        robot.flappers.setPosition(Globals.flapperClose);
        sleep(200);
        robot.stackServo.setPosition(Globals.stackInit);
        sleep(150);
        robot.Arm.setPosition(Globals.ArmInit);
        sleep(200);
        setServoShoulder(Globals.shoulderInit);
        robot.rotate.setPosition(Globals.rotateInit);
        robot.switchPixel.setPosition(Globals.switchPixelInit);
        robot.droneLock.setPosition(Globals.DroneLock);
        robot.ratchet.setPosition(Globals.RachetOpen);

        while(opModeInInit())
        {

            gamepad1.rumble(1, 1, 400);


            if(gamepad1.dpad_up){
                ElevatorPos += 1;
            } else if (gamepad1.dpad_down) {
                ElevatorPos -=1;
            } else if (gamepad1.start) {
                ElevatorPos=0;
            }
//            Todo Elevator adjustment in init.
            if(gamepad1.dpad_left){
                sliderPos += 1;
            } else if (gamepad1.dpad_right) {
                sliderPos -=1;
            } else if (gamepad1.start) {
                sliderPos=0;
            }
            Elevator.extendTo(ElevatorPos,0.9);
            Extension(sliderPos,1);

        }

        Elevator.reset();
        Extension.reset();

        // TODO ============================================ Single Pixel Drop ===========================================================

       //// TODO Toggle Logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_LEFT_OPEN),
                new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_RIGHT_OPEN)
        );

        // TODO ============================================= Ground Level Intake and Intake OFF =====================================================
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_OFF),
                new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.INTAKE_ON)
        );


    }

    @Override
    public  void run()
    {
        super.run();


        // TODO ============================================ Elevator Height  ===========================================================
        if(gamepad2.dpad_up && timer.seconds() > 0.2){
            if(ElevatorCounter<9)
            {
                ElevatorCounter++;
                timer.reset();
            }

        }
        if(gamepad2.dpad_down && timer.seconds() > 0.2){
            timer.reset();
            if(ElevatorCounter>=1)
            {
                ElevatorCounter--;
            }
        }
        // TODO =============================================  CHANGE ELEVATOR POS LOGIC ===============================================
        if(gamepad2.dpad_up && isElevate){
            if(ElevatorCounter<9)
            {
                if(ElevatorCounter == 0){
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HOME, 0));
                }
                if (ElevatorStates[ElevatorCounter - 1] == 1) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.ONE, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 2) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.TWO, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 3) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.THREE, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 4) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.FOUR, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 5) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.FIVE, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 6) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.SIX, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 7) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.SEVEN, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 8) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.EIGHT, 0));
                } else if (ElevatorStates[ElevatorCounter - 1] == 9) {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.NINE, 0));
                } else {
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HOME, 0));
                }
            }
            else {
                schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.HOME, 0));
            }

        }
        if(gamepad2.dpad_down && isElevate)
        {
            if(ElevatorCounter>=1){
                if(ElevatorCounter == 0){
                    schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HOME, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 1)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.ONE, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 2)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.TWO, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 3)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.THREE, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 4)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.FOUR, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 5)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.FIVE, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 6)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.SIX, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 7)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.SEVEN, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 8)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.EIGHT, 0));
                }
                else if (ElevatorStates[ElevatorCounter - 1] == 9)
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.NINE, 0));
                }
                else
                {
                    schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.HOME, 0));
                }
            }
            else {
                schedule(new ElevatorCommand(Elevator,ElevatorSubsytem.ElevateState.HOME, 0));
            }
        }

        // TODO ============================================ Intake Position ===========================================================
        if(gamepad1.right_bumper && !isDrop){
            if(robot.beam1.getState()==false && robot.beam2.getState()==false){
                schedule( new TransferSeq(Intake, Outtake, Elevator, Extension),
                        new WaitCommand(100),
                        new InstantCommand(()->gamepad1.rumble(1, 1, 400)
                        ));
            }
            else{
                head = 0.5;
                strafe = 0.3;
                straight = 0.8;
                schedule( new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_OPEN),
                        new WaitCommand(100),
                        new IntakePosSeq(Outtake, Intake,Elevator)
                );
                extendit = FALSE;
                isIntake = FALSE;
                isElevate = false;
                shiftPixel = false;
                beamFlag = true;
            }
        }


        // TODO ============================================ REVERSE INTAKE ===========================================================
        if(gamepad2.b){
            schedule(  new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.INTAKE_DOWN, IntakeSubsystem.RollerIntakeState.PIXEL_OUT)); // Intaking the Pixel From ground
        }
        if(gamepad1.b){
            head = 0.5;
            strafe = 0.3;
            straight = 0.5;
        }

       // TODO ============================================ Transfer ===========================================================
      // Transferring the pixel when colour sensor gets active
        if((robot.beam1.getState() == false && robot.beam2.getState() == false)){

            telemetry.addLine("Im inside beam");
            if(extensionFlag && robot.IntakeExtensionLeft.getCurrentPosition()<100 && beamFlag)
            {
                telemetry.addLine("Pluunge seq");

                schedule( new TransferSeq(Intake, Outtake, Elevator, Extension),
                        new WaitCommand(100),
                        new InstantCommand(()->gamepad1.rumble(1, 1, 400)
                        ));
                head = 0.5;
                strafe = 0.3;
                straight = 0.8;
                isDrop = true;
                beamFlag = false;
            }
        }

        //////// Transferring logic ends
        if(gamepad2.y) // Forcefully Executing the Transfer Sequence
        {
            if(robot.IntakeExtensionLeft.getCurrentPosition()<100  && beamFlag==false){
                schedule( new TransferSeq(Intake, Outtake, Elevator, Extension),
                        new WaitCommand(100),
                        new InstantCommand(()->{
                            gamepad1.rumble(1, 1, 400);
                        }
                        ));
                head = 0.5;
                strafe = 0.3;
                straight = 0.8;
                isDrop = true;
            }
        }

        // TODO ============================================================ RESET ========================================================================
        if(gamepad1.back){
            extendit = false;
            isIntake = false;
            isDrop = true;
            extensionFlag = true;
            beamFlag = false;
        }

        // TODO ============================================================== DROPPING SEQUENCE ===========================================================
        if(gamepad1.left_bumper && isDrop)
        {
            head = 0.5;
            strafe = 0.3;
            straight = 0.5;

            // DROPPING SEQ

            if(ElevatorCounter==0){
                schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.HOME, storeState));
            }

            else if(ElevatorCounter>=1){
               if (ElevatorStates[ElevatorCounter - 1] == 1)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.ONE, storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 2)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.TWO , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 3)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.THREE , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 4)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.FOUR , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 5)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.FIVE , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 6)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.SIX , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 7)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.SEVEN , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 8)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.EIGHT , storeState));
               }
               else if (ElevatorStates[ElevatorCounter - 1] == 9)
               {
                   schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.NINE , storeState));
               }
           }
            else {
                schedule(new DropSeq(Intake,Outtake, Elevator, ElevatorSubsytem.ElevateState.HOME, storeState));
            }

            // DROPPING SEQ END

            extendit = true;
            isIntake = true;
            isDrop = false;
            isElevate = true;
            shiftPixel = true;
        }

        if(gamepad1.dpad_up){
            inc(1);
        }
        if(gamepad1.dpad_down){
            dec(1);
        }


        // TODO ============================================ Both Pixel Drop ===========================================================
        if(gamepad1.right_trigger>0)
        {
            schedule( new GripperCommand(Outtake, OutakeSubsystem.GripperState.GRIP_OPEN));
        }

        // TODO ============================================ NULL ===========================================================
        if(gamepad1.left_trigger>0)
        {

        }

//        // TODO ============================================ Drone Shoot ===========================================================
        if(gamepad2.right_stick_button)
        {
            robot.droneLock.setPosition(Globals.DroneOPen);
        }

        // TODO ================================================= STACK PIXEL ========================================================
        if(gamepad2.a){
            schedule( new IntakeServoCommand(Intake, IntakeSubsystem.IntakeServoState.FIVE_PIXEL));
            schedule(  new IntakePixel(Intake, IntakeSubsystem.IntakeServoState.FIVE_PIXEL, IntakeSubsystem.RollerIntakeState.INTAKE_ON));

        }


//        // TODO ============================================ Hanging ===========================================================
        if(gamepad2.x)
        {
            // Hang position
            schedule(new GripperCommand(Outtake,OutakeSubsystem.GripperState.GRIP_OPEN),
                    new DropSeq(Intake,Outtake));
            schedule(new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HANGERPOS,dropHeightOne));

        }
        if(gamepad2.right_trigger>0)
        {
            // Hang
            robot.ratchet.setPosition(Globals.RachetClose);
            schedule( new ElevatorCommand(Elevator, ElevatorSubsytem.ElevateState.HANG, dropHeightOne),
                    new InstantCommand(()-> Intake.setIntakeServo(Globals.stackInit))
            );

            Extension.Extension(50, 0.4);
        }

        // TODO ================================================= Ratchet Open ===============================================================
        if(gamepad2.back){
            robot.ratchet.setPosition(Globals.RachetOpen);
        }



        // TODO ============================================ X Extension  ===========================================================
        if(gamepad1.x)
        {
            // Hang position
            schedule( new ExtensionCommand(Extension, ExtensionSubsystem.IntakeExtensionState.INIT));
            extensionFlag = true;
        }
        if(gamepad1.a && extensionFlag)
        {
            // Hang
            schedule( new ExtensionCommand(Extension, ExtensionSubsystem.IntakeExtensionState.Extend));
            extensionFlag = false;
        }


        // TODO ========================================== SWITCH PIXEL =======================================================

        if(gamepad2.left_trigger>0.8){
            inversePixelFlag=true;
        }
        else if (gamepad2.left_trigger<0.3)
        {
            inversePixelFlag=false;
        }

        if (gamepad2.right_bumper)
        {
            if (inversePixelFlag) {
                storeState= OutakeSubsystem.SwitchPixelState.SWITCH_VERTICAL_INVERSE;
                if(Elevator.elevateState.toString()!="HOME") {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_VERTICAL_INVERSE));
                }
            } else if(!inversePixelFlag) {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_VERTICAL;
                if(Elevator.elevateState.toString()!="HOME") {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_VERTICAL));
                }
            }
        }

        if (gamepad2.left_bumper) {
            if (inversePixelFlag) {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_HORIZONTAL_INVERSE;
                if(!Elevator.elevateState.toString().equals("HOME")) {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_HORIZONTAL_INVERSE));
                }

            } else if(!inversePixelFlag) {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_HORIZONTAL;
                if(!Elevator.elevateState.toString().equals("HOME")) {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_HORIZONTAL));
                }
            }
        }

        if (gamepad2.dpad_left) {
            if (inversePixelFlag) {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_LEFT_INVERSE;
                if(!Elevator.elevateState.toString().equals("HOME")) {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_LEFT_INVERSE));
                }
            } else if(!inversePixelFlag)  {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_LEFT;
                if(Elevator.elevateState.toString()!="HOME") {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_LEFT));

                }
            }
        }

        if (gamepad2.dpad_right) {
            if (inversePixelFlag) {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_RIGHT_INVERSE;
                if(Elevator.elevateState.toString()!="HOME") {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_RIGHT_INVERSE));
                }
            } else if(!inversePixelFlag)  {
                storeState=OutakeSubsystem.SwitchPixelState.SWITCH_RIGHT;
                if(Elevator.elevateState.toString()!="HOME") {
                    schedule(new SwitchPixelCommand(Outtake, OutakeSubsystem.SwitchPixelState.SWITCH_RIGHT));
                }
            }
        }


        // TODO ======================================= PIXEL SHIFTING ========================================================
        if(gamepad2.left_stick_button && shiftPixel) {
            schedule(new SwitchPixelCommand(Outtake, Outtake.switchPixelVertical),
                    new ArmCommand(Outtake, OutakeSubsystem.ArmState.DROP)
                    );
            shiftPixel = false;
        }





        // TODO ============================================ Drive ===========================================================
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y *  straight,
                        -gamepad1.left_stick_x * strafe,
                        -gamepad1.right_stick_x * head
                )
        );

        telemetry.addData("Heading:",drive.robotHeading);


        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("Left Motor POS", robot.leftElevator.getCurrentPosition());
        telemetry.addData("Right Motor POS", robot.rightElevator.getCurrentPosition());
        telemetry.addData("Extension POS", robot.IntakeExtensionLeft.getCurrentPosition());
        telemetry.addData("Left Motor Current", robot.leftElevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Motor Current", robot.rightElevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Height" , ElevatorCounter);
        telemetry.addData("Beam1",robot.beam1.getState());
        telemetry.addData("Beam2",robot.beam2.getState());
        telemetry.addData("Flag",beamFlag);
        telemetry.addData("extensionFlag",extensionFlag);
        telemetry.update();

    }

    public void setServoShoulder(double leftPos){    //Todo add servo offset if needed.
        double rightPos=1-leftPos;
        robot.leftShoulder.setPosition(leftPos);
        robot.rightShoulder.setPosition(rightPos);
    }

    public void Extension(int ExtendVal, double pow)
    {
        robot.IntakeExtensionLeft.setTargetPosition(ExtendVal);
        robot.IntakeExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.IntakeExtensionLeft.setPower(pow);
    }

    public void inc(double power){

        robot.leftElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()-55);
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()-55);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);
    }


    public void dec(double power){

        robot.leftElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()+55);
        robot.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftElevator.setPower(power);

        robot.rightElevator.setTargetPosition(robot.leftElevator.getCurrentPosition()+55);
        robot.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightElevator.setPower(power);
    }

}
