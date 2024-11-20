package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    // Drone on 0.65
    public static boolean IS_AUTO=false;
    public static boolean IS_IMU=false;

    // TODO==========================================================  FOR NAV X ====================================================
    public static boolean IS_CUSTOMIMU=false;


    // TODO==========================================================  Elevator Value ====================================================
    public static int incVal= -27;
    public static int lifterDown=0;
    public static  int lifterOne= -375+incVal;
    public static  int liftYellow= -650+incVal;
    public static  int lifterTwo= -750+incVal;
    public static  int lifterThree= -1075;
    public static int lifterFour= -1470;
    public static int lifterFive= -1790;
    public static int lifterSix = -2100;
    public static int lifterSeven = -2400;
    public static int lifterEight = -2700;
    public static int lifterNine = -3050;
    // TODO==========================================================  Hanging Value ====================================================


    public static  int hangpos = -2400;

    public static int hang = -500;


    // TODO==========================================================  Intake Servo Value ====================================================
    public static double stackInit=1;//0.8405;
    public static double stackDown=0.638;//0.56;//0.444;
    public static double stackThree=0.5616;
    public static double stackFour=0.608;
    public static double stackFive=0.6338;


    // TODO==========================================================  FLapper Servo Value ====================================================

    //Intake values
    public static double flapperOpen=0.88;
    public static double flapperMid=0.358;

    public static double flapperClose=0.2;//0.28B;// 0.25


    // TODO==========================================================  Shoulder Servo Value ====================================================
    public static double shoulderInit=0.745;//0.714;//0.8166;
    public static double shoulderSafePick= 0.7;
    public static double shoulderSafePick1= 0.65;
    public static double shoulderSafePick2= 0.7;
    public static double shoulderPick= 0.9216;
    public static double shoulderPrePick= 1;
    public static double shoulderDrop= 0.301;//0.3000;//0.34;
    public static double shoulderSafeDrop= 1;
    public static double shoulderPreDrop= 1;

    // TODO==========================================================  Switch Pixel Servo Value ====================================================
    public static double switchPixelInit=0.5105;//0.505;//0.520;  //Safe   0.49
    public static double switchPixelDrop= 0.5255; //0.49;//0.520;  //Safe   0.49
    public static double switchPixelLeft=0.3655;
    public static double switchPixelLeftInverse=0.9288;
    public static double switchPixelRight=1-switchPixelLeft;  //USE EQN 1- PEXEL LEFT
    public static double switchPixelRightInverse= 1-switchPixelLeftInverse;
    public static double switchPixelHorizontal=0.51;
    public static double switchPixelHorizontalInverse=0;
    public static double switchPixelVertical=0.7977;
    public static double switchPixelVerticalInverse=1-switchPixelVertical;
    public static double switchPixelIntake=0.4738;



    // TODO==========================================================  Left / Right Gripper Servo Value ====================================================

    //Gripper
    public static double leftGripOpen=0.2727;
    public static double leftGripSafe=0.5205; // Has To Change
    public static double leftGripClose=0.6866;

    public static double rightGripOpen=0.7516;
    public static double rightGripSafe=0.501; // Has To Change
    public static double rightGripClose=0.2338;


    // TODO==========================================================  Rotate Servo Value ====================================================

    public static double rotateInit=0.5;//0.426;
    public static double rotatePick=0.5;
    public static double rotateDrop= 0.5166; //0.4855;
    public static double rotatePreDrop=0.5;


    // TODO=============================================================== x Extension Value ===================================================
    public static int Extend = -1600;
    public static int ExtendInit = 0;
    public static int Pull_In = 50;



    // TODO =================================================================  ARM Value  ====================================================

    public static double ArmInit=0.87166 ;//0.623;
    public static double ArmMid=0.8877;
    public static double ArmPick=0.8466;//0.831;//0.81055;
    public static double ArmSafePick=0.830;//0.9155;
    public static double ArmPrePick=0.851666;
    public static double ArmDrop=0.392;//0.44;//0.3966;
    public static double ArmSafeDrop=0.8655;
    public static double ArmPreDrop=0.873888;

    // TODO ================================================================ OPENcv ====================================================================
    public static Location SIDE = Location.CLOSE;
    public static Location ALLIANCE = Location.BLUE;

    // TODO ================================================================ RACHET VALUES ================================================================

    public  static double RachetOpen = 0.78;
    public  static double RachetClose = 0.22;

    public  static double DroneLock = 0.65;
    public  static double DroneOPen = 0.14;


    // TODO =========================================================== AUTO POS ===========================================================================


    public static double shoulderYellowSafe= 0.45;
    public static double autoSwitchInit=0.8;
    //Auto Grip
    public static double autoleftGripInit=0.88;
    public static double autorightGripInit=0.22;
    //Auto Arm
    public static double autoArmInit=0.251;
    public static double autoArmYellow=0.4188;
    public static double autostackInit=0.6999;
}





