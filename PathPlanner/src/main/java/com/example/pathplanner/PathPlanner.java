package com.example.pathplanner;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class PathPlanner {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

          Pose2d startPose = new Pose2d(14, 62, Math.toRadians(180)); // original

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(13.5,16.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 13.13)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .lineToConstantHeading(new Vector2d(15, 21))  //center
                                        .lineToConstantHeading(new Vector2d(46.2,36))//
                                        .lineToConstantHeading(new Vector2d(48,36))
                                        .lineToConstantHeading(new Vector2d(24,57))
                                        .lineToConstantHeading(new Vector2d(-38,57))
                                        .lineToLinearHeading(new Pose2d(-59, 40, Math.toRadians(220))) // -56, 36




//                                        .lineToConstantHeading(new Vector2d(26 , -42))
//                                        .splineToConstantHeading(new Vector2d(53,-34),0)
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-58,-12),Math.PI)
//                                        .setReversed(false)
//                                        .splineToConstantHeading(new Vector2d(10,-12),0)
//                                        .splineToConstantHeading(new Vector2d(53,-34),0)
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-58,-12),Math.PI)
//                                        .setReversed(false)
//                                        .splineToConstantHeading(new Vector2d(10,-12),0)
//                                        .splineToConstantHeading(new Vector2d(53,-34),0)
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-40,-12),Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-58,-24),Math.PI)
//                                        .setReversed(false)
//                                        .splineToConstantHeading(new Vector2d(-40,-12),0)
//                                        .splineToConstantHeading(new Vector2d(10,-12),0)
//                                        .splineToConstantHeading(new Vector2d(53,-34),0)
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-40,-12),Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-58,-24),Math.PI)
//                                        .setReversed(false)
//                                        .splineToConstantHeading(new Vector2d(-40,-12),0)
//                                        .splineToConstantHeading(new Vector2d(10,-12),0)
//                                        .splineToConstantHeading(new Vector2d(53,-34),0)
                                        .build()
                        );






//                                .lineToConstantHeading(new Vector2d(-6, 32))














//                                .lineToLinearHeading(new Pose2d(12,38,Math.toRadians(90))) // centerdrop pose
//          .lineToLinearHeading(new Pose2d(35,36,Math.toRadians(180)))  //apriltag detect and reset pose
//
//          .lineToConstantHeading(new Vector2d(-55,36))
//          .lineToConstantHeading(new Vector2d(48,36))
//          .lineToConstantHeading(new Vector2d(-42,34))
//          .lineToConstantHeading(new Vector2d(-55,24))
//          .lineToConstantHeading(new Vector2d(-42,36))
//          .lineToConstantHeading(new Vector2d(48,32))
////                                .splineToLinearHeading(new Pose2d(18.6, 42, Math.toRadians(-90)), Math.toRadians(-90))
////                                .lineToLinearHeading(new Pose2d(30, 35, Math.toRadians(-180)))
//


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}