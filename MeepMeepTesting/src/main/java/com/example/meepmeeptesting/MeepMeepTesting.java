package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();


        //backstage blue:
        //new Pose2d(12,60.5,Math.toRadians(270));

        //frontstage blue:
        //new Pose2d(-36,60.5,Math.toRadians(270));




        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6,60.5,Math.toRadians(270)))
                //clip and go to basket

                .lineToY(32)
                //clip and intake sample
                .strafeToLinearHeading(new Vector2d(57.5,57.5), Math.toRadians(48))
                //output sample
                .strafeToSplineHeading(new Vector2d(52,41), Math.toRadians(253))
                //intake sample
                .strafeToLinearHeading(new Vector2d(57.5,57.5), Math.toRadians(48))
                //output sample
                .strafeToSplineHeading(new Vector2d(55.5,40), Math.toRadians(285))
                //intake sample
                .strafeToLinearHeading(new Vector2d(57.5,57.5), Math.toRadians(48))
                //output sample
                .strafeToSplineHeading(new Vector2d(57,38), Math.toRadians(318))
                //intake sample
                .strafeToLinearHeading(new Vector2d(57.5,57.5), Math.toRadians(48))
                //output sample
                .strafeToLinearHeading(new Vector2d(3,32), Math.toRadians(270))
                /*
                //go to push
                .strafeToLinearHeading(new Vector2d(-32,36.5), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(-38,10), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(-44.5,12), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(-57.5,57.5), Math.toRadians(105))
                //intake sample during this (put down and spin intake)


                .strafeToLinearHeading(new Vector2d(-46.5,12), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(-60,57.5), Math.toRadians(105))
                //intake sample
                */



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}