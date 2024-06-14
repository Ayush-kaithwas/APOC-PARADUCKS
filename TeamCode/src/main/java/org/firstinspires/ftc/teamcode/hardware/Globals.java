package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.Locations;

import java.security.PublicKey;

@Config
public class Globals {

    public static Locations SIDE = Locations.BLUE;
    public static boolean IS_AUTO=false;
    public static boolean IS_IMU=false;

    // TODO==========================================================  FOR NAV X ====================================================
    public static boolean IS_CUSTOMIMU=false;


    // TODO==========================================================  Elevator Value ====================================================
    public static int incVal= -27;
    public static int lifterDown=0;
    public static  int lifterOne= -700+incVal;
    public static  int liftYellow= -650+incVal;
    public static  int lifterTwo= -750+incVal;
    public static  int lifterThree= -880;
    public static int lifterFour= -1010;
    public static int lifterFive= -1180;
    public static int lifterSix = -1410;
    public static int lifterSeven = -1600;
    public static int lifterEight = -1800;
    public static int lifterNine = -2100;
    // TODO==========================================================  Hanging Value ====================================================


    public static  int hangpos = -1200;

    public static int hang = 0;


    // TODO==========================================================  Intake Servo Value ====================================================
    public static double stackInit=0.8466;//0.8405;
    public static double stackDown=0.444;
    public static double stackThree=0.5616;
    public static double stackFour=0.608;
    public static double stackFive=0.6338;


    // TODO==========================================================  FLapper Servo Value ====================================================

    //Intake values
    public static double flapperOpen=0.65;
    public static double flapperMid=0.358;
    public static double flapperClose=0.0938;


    // TODO==========================================================  Shoulder Servo Value ====================================================
    public static double shoulderInit=0.8166;//0.623;
    public static double shoulderSafePick= 0.65;
    public static double shoulderPick= 1;
    public static double shoulderPrePick= 1;
    public static double shoulderDrop= 0.3000;//0.34;
    public static double shoulderSafeDrop= 1;
    public static double shoulderPreDrop= 1;

    // TODO==========================================================  Switch Pixel Servo Value ====================================================
    public static double switchPixelInit=0.5105;//0.505;//0.520;  //Safe   0.49
    public static double switchPixelDrop= 0.49;//0.520;  //Safe   0.49
    public static double switchPixelLeft=0.588;
    public static double switchPixelLeftInverse=0.0489;
    public static double switchPixelRight=0.437;  //USE EQN 1- PEXEL LEFT
    public static double switchPixelRightInverse= 0.951;
    public static double switchPixelHorizontal=0.7866;
    public static double switchPixelHorizontalInverse=0.22666;
    public static double switchPixelVertical=0.520;
    public static double switchPixelVerticalInverse=0.102;
    public static double switchPixelIntake=0.4738;



    // TODO==========================================================  Left / Right Gripper Servo Value ====================================================

    //Gripper
    public static double leftGripOpen=0.76;
    public static double leftGripSafe=0.5; // Has To Change
    public static double leftGripClose=0.201;

    public static double rightGripOpen=0.182;
    public static double rightGripSafe=0.5; // Has To Change
    public static double rightGripClose=0.707;


    // TODO==========================================================  Rotate Servo Value ====================================================

    public static double rotateInit=0.5;//0.426;
    public static double rotatePick=0.5;
    public static double rotateDrop=0.4855;//0.88388;
    public static double rotatePreDrop=0.4655;


    // TODO=============================================================== x Extension Value ===================================================
    public static int Extend = -1600;
    public static int ExtendInit = 0;

    // TODO =================================================================  ARM Value  ====================================================

    public static double ArmInit=0.87166 ;//0.623;
    public static double ArmMid=0.8877;
    public static double ArmPick=0.831;//0.81055;
    public static double ArmSafePick=0.851666;
    public static double ArmPrePick=0.851666;
    public static double ArmDrop=0.44;//0.3966;
    public static double ArmSafeDrop=0.8655;
    public static double ArmPreDrop=0.873888;




}




/*Init
 * Rotate = 0.5
 * Flapper = 0/ Change
 * ARM = 0.87166     safe 0.851666  new safe         pick 0.831        afterpick safe-0.8377  afterafterpick pixel doent crash-0.873888
 * Switch Pixel = 0.4755
 * Right S  = 0.18277 init   0pick     drop
 * Left S = 0.8166           1          drop
 * stack-0.8466
 *
 *
 * gripL  0.860
 * gripR  0.397
 *

 * */







