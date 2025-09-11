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

        /// Scenario #1 --> Red Far Side GPP
        RoadRunnerBotEntity redFarGPP = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redFarGPP.runAction(redFarGPP.getDrive().actionBuilder(new Pose2d(60, 20, Math.toRadians(180)))
                .waitSeconds(3)
                .splineToLinearHeading(
                        new Pose2d(35, 35, Math.toRadians(90)),
                        Math.toRadians(90)
                )
                .lineToY(50)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(-45))
                .build());

        /// Scenario #2 --> Red Far Side PGP
        RoadRunnerBotEntity redFarPGP = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redFarPGP.runAction(redFarGPP.getDrive().actionBuilder(new Pose2d(60, 20, Math.toRadians(180)))
                .waitSeconds(3)
                .splineToLinearHeading(
                        new Pose2d(50, 56, Math.toRadians(45)),
                        Math.toRadians(90)
                )
                .splineToLinearHeading(new Pose2d(60, 56, Math.toRadians(45)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(0))
                .build());

        /// Scenario #3 --> Red Far Side PPG
        RoadRunnerBotEntity redFarPPG = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redFarPPG.runAction(redFarGPP.getDrive().actionBuilder(new Pose2d(60, 20, Math.toRadians(180)))
                .waitSeconds(3)
                .setTangent(90)
                .splineToLinearHeading(
                        new Pose2d(40, 56, Math.toRadians(-135)),
                        Math.toRadians(-90)
                )
                .lineToY(35)
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(180)), Math.toRadians(0))
                .build());


        ///scenario 4: red close launch zone green, purple, purple
        RoadRunnerBotEntity redCloseGPP = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redCloseGPP.runAction(redCloseGPP.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .lineToX(-55)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(-22, 55, Math.toRadians(330)), Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .build());



        /// Scenario 5: red close, purple purple green
        RoadRunnerBotEntity redClosePPG = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        redClosePPG.runAction(redClosePPG.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .lineToX(-55)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(-17, 37, Math.toRadians(60)), Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .build());

        /// scenario 6: red close, purple green purple
        RoadRunnerBotEntity redClosePGP = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        redClosePGP.runAction(redClosePGP.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .lineToX(-55)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(12, 40, Math.toRadians(90)), Math.toRadians(100))
                .waitSeconds(1)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redFarGPP)
                .addEntity(redFarPGP)
                .addEntity(redFarPPG)

                .addEntity(redClosePGP)
                .addEntity(redCloseGPP)
                .addEntity(redClosePPG)
                .start();
    }
}