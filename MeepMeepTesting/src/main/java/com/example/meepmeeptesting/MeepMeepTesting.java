package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        // for BlueSideBasket
    //    /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-16, -60, Math.toRadians(270)))
                        .setReversed(true)

                        .splineTo(new Vector2d(-8, -30), Math.toRadians(90))

                        .waitSeconds(3)

                        .lineTo(new Vector2d(-8,-40))

                        .turn(Math.toRadians(180))

                        .setReversed(false)

                        .strafeLeft(42)

                        .waitSeconds(1)

                        .lineToSplineHeading(new Pose2d(-60, -53, Math.toRadians(45)))

                        .waitSeconds(1)

                        .turn(Math.toRadians(45))

                        .waitSeconds(1)

                        .turn(Math.toRadians(-45))

                        .waitSeconds(1)

                        .turn(Math.toRadians(60))

                        .waitSeconds(1)

                        .turn(Math.toRadians(-60))

                        .build());

//         */

        // for observation zone
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(16, -60, Math.toRadians(270)))
                        .setReversed(true)

                        .lineTo(new Vector2d(16,-32))

                        .waitSeconds(3)

                        .lineTo(new Vector2d(16,-40))

                        .turn(Math.toRadians(-90))

                        .setReversed(false)

                        .lineTo(new Vector2d(25,-40))

                        .waitSeconds(1)

                        .lineToSplineHeading(new Pose2d(25, -10, Math.toRadians(180)))


                        .build());
         */


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}