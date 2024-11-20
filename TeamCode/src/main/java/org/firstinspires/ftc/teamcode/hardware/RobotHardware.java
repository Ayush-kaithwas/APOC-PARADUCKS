package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {
    //Todo ======================================================== Intake Actuators ===================================================================
    public  DcMotorEx intakeMotor=null;//Roller Intake.
    public  Servo stackServo=null;
    public  Servo flappers=null;

    //Todo ========================================================= Outake Actuators ===================================================================

    /* Shoulder */
    public  Servo leftShoulder=null;
    public  Servo rightShoulder=null;
    public  Servo rotate=null;

    /* Switch Pixel */
    public  Servo switchPixel=null;

    // ARM
    public Servo Arm = null;

    /* Gripper Servo */
    public  Servo leftGrip=null;
    public  Servo rightGrip=null;

    //Todo ========================================================= Elevator Actuators ===================================================================
    public DcMotorEx leftElevator=null;
    public DcMotorEx rightElevator=null;
    public Servo ratchet=null;

    //Todo ========================================================= Intake Actuators ===================================================================

    public DcMotorEx IntakeExtensionLeft=null;

    //Todo ========================================================= Drone Servo ===================================================================
    public Servo droneLock=null;

    //Todo ========================================================= Color Sensor ===================================================================
    public RevColorSensorV3 sensorColor1;
    public RevColorSensorV3 sensorColor2;

    //Todo ========================================================= Navx ===================================================================
    NavxMicroNavigationSensor Navx;
    IntegratingGyroscope gyro;

    //Todo ========================================================= BEAM Breaker ===================================================================
    public DigitalChannel beam1=null;
    public  DigitalChannel beam2=null;



    //Todo ========================================================= Robot Setup ===================================================================
    private static RobotHardware instance = null;    // ref variable to use robot hardware
    public boolean enabled;                          //boolean to return instance if robot is enabled.

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public HardwareMap hardwareMap;

    //Todo init() for hardware map
    //Call this method inside auto and teleop classes to instantly hardware map all actuators.
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        //Todo ========================================================= Map Intake Actuators ===================================================================
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        stackServo = hardwareMap.get(Servo.class, "stackS");
        flappers = hardwareMap.get(Servo.class, "flaps");

        //Todo ========================================================= Map Outtake Actuators ===================================================================
        leftShoulder = hardwareMap.get(Servo.class, "leftS");
        rightShoulder = hardwareMap.get(Servo.class, "rightS");
        rotate = hardwareMap.get(Servo.class, "rotate");
        /* Switch Pixel */
        switchPixel = hardwareMap.get(Servo.class, "switchP");
        /* Gripper Servo */
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        Arm = hardwareMap.get(Servo.class, "arm");

        //Todo ========================================================= Map Hanger/Elevator Actuators ===================================================================
        ratchet = hardwareMap.get(Servo.class, "ratchet");

        rightElevator = hardwareMap.get(DcMotorEx.class, "elevatorM1");
        leftElevator = hardwareMap.get(DcMotorEx.class, "elevatorM2");


        //Todo ========================================================= Map Drone Actuators ===================================================================
        droneLock = hardwareMap.get(Servo.class, "drone");


        //Todo ========================================================= Map Intake Extension Actuators ===================================================================
        IntakeExtensionLeft = hardwareMap.get(DcMotorEx.class, "sliderM1");


        //Todo ========================================================= Color Sensor ===================================================================
        sensorColor1 = hardwareMap.get(RevColorSensorV3.class, "cs1");
        sensorColor2 = hardwareMap.get(RevColorSensorV3.class, "cs2");

        //Todo ========================================================= BEAM Breaker ===================================================================
        beam1=hardwareMap.get(DigitalChannel.class,"beam1");
        beam2=hardwareMap.get(DigitalChannel.class,"beam2");

        //Todo ========================================================= Setting Mode for all the Actuators ===================================================================
//        leftElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtensionLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




    }
}
