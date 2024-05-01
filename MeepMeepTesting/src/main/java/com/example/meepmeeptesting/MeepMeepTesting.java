package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        //backstage blue:
        //new Pose2d(12,60.5,Math.toRadians(270));

        //frontstage blue:
        //new Pose2d(-36,60.5,Math.toRadians(270));

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12,60.5,Math.toRadians(270)))
                //left
                .strafeToLinearHeading(new Vector2d(30.5,33.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(34,43), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(54.5,40.5))

                //center
                //.lineToYConstantHeading(32.25)

                //right
                //.strafeToLinearHeading(new Vector2d(8.5, 33.5), Math.toRadians(180))


                .strafeToLinearHeading(new Vector2d(48, 12), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}