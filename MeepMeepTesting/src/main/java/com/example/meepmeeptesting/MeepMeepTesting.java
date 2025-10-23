package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        /// red far solo auto
        RoadRunnerBotEntity redFarSolo = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redFarSolo.runAction(redFarSolo.getDrive().actionBuilder(new Pose2d(60, 20, Math.toRadians(180)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(35, 35, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(-45))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(15, 35, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(-45))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-15, 35, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(-45))
                .build());

        /// Red far side partner
        RoadRunnerBotEntity redFarPartner = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redFarPartner.runAction(redFarPartner.getDrive().actionBuilder(new Pose2d(60, 20, Math.toRadians(180)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(50, 56, Math.toRadians(45)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(60, 56, Math.toRadians(45)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, 35, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(-45))
                .waitSeconds(1)
                .build());


        ///scenario 4: red close solo
        RoadRunnerBotEntity redCloseSolo = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redCloseSolo.runAction(redCloseSolo.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-16, 35, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(15, 35, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(35, 35, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(90))
                .build());

//
//
        //red close partner
        RoadRunnerBotEntity redClosePartner = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        redClosePartner.runAction(redClosePartner.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-17, 37, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                 .waitSeconds(2)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(0, 54, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(5, 40, Math.toRadians(0)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(90))
                .build());

//

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(redFarSolo)
//                .addEntity(redCloseSolo)
                .addEntity(redFarPartner)
//                .addEntity(redClosePartner)
//
                .start();
    }
}