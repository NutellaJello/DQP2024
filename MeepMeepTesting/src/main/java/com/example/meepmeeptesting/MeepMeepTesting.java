package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -35, Math.toRadians(90)))
                       // from start goes to place, splines to place brick

                        .splineTo(new Vector2d(-34, 0), 0)

                        .back(5)

                       .splineTo(new Vector2d(-55, -55), Math.toRadians(230))

                        // second cycle. has to turn or else really wonky.
                        .turn(Math.toRadians(40))

                        .splineTo(new Vector2d(-34, 0), 0)

                        .back(5)

                        .splineTo(new Vector2d(-55, -55), Math.toRadians(230))

                        // Third cycle
                        .turn(Math.toRadians(40))

                        .splineTo(new Vector2d(-34, 0), 0)

                        .back(5)

                        .splineTo(new Vector2d(-55, -55), Math.toRadians(230))

                        // 4th cycle
                        .turn(Math.toRadians(40))

                        .splineTo(new Vector2d(-34, 0), 0)

                        .back(5)

                        .splineTo(new Vector2d(-55, -55), Math.toRadians(230))



                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}