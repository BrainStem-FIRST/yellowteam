package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
                //scenario 4: red close launch zone green, purple, purple
        RoadRunnerBotEntity redCloseGPP = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redCloseGPP.runAction(redCloseGPP.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .waitSeconds(3)
                .lineToX(-55)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(-22, 55, Math.toRadians(330)), Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .build());



//      scenario 5: red close, purple purple green
        RoadRunnerBotEntity redClosePPG = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        redClosePPG.runAction(redClosePPG.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .waitSeconds(3)
                .lineToX(-55)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(-17, 37, Math.toRadians(60)), Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .build());

        //scenario 6: red close, purple green purple
        RoadRunnerBotEntity redClosePGP = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        redClosePGP.runAction(redClosePGP.getDrive().actionBuilder(new Pose2d(-60, 40, Math.toRadians(135)))
                .waitSeconds(3)
                .lineToX(-55)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(12, 40, Math.toRadians(90)), Math.toRadians(100))
                .waitSeconds(3)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-55, 40, Math.toRadians(135)), Math.toRadians(135))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redClosePGP)
                .addEntity(redCloseGPP)
                .addEntity(redClosePPG)
                .start();
    }
}