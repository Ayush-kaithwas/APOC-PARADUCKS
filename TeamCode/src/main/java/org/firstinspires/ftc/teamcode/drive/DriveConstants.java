package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    //TODO GOBILDA 5203 13.7:1
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 6000;

    //Note: Rev & Gobilda have same MAX_RPM

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 0.07299; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH =14.65;//13.375;@3.14 ang velo//


    public static double kV =  0.0124; //0.01229;// 0.01230;////0.01189;//pd-0.01180//B0.01259
    public static double kA =0.0035;//0.0032 ; //pd-0.002//0.0035mightbe best
    public static double kStatic =0.00016; //0.0305;//0.01;//0.09986;// 0.0014; //pd-0.08197//B 0.01

    //use this og is better
    public static double MAX_VEL = 78; // 50
    public static double MAX_ACCEL = 78;
    public static double MAX_ANG_VEL = Math.toRadians(130.336);
    public static double MAX_ANG_ACCEL = 2.7;//2.9//Math.toRadians(130.336);

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
