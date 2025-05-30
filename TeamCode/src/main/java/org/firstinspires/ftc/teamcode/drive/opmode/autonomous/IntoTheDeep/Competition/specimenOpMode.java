package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Competition;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BlueAuto", group="competition")
public class specimenOpMode extends LinearOpMode {
    private boolean pathFinished = false;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-35, -60, Math.toRadians(90)));
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap, telemetry);
        Pivot pivot = new Pivot(hardwareMap, telemetry);

        Action firstStage1 = drive.actionBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(40,-40), Math.toRadians(270))
                .waitSeconds(0.1)
                .build();
        // Move to loading zone, and turn around

        Action firstStage2 = drive.actionBuilder(new Pose2d(40,-40, Math.toRadians(270)))
                .lineToY(-50)
                .waitSeconds(0.1)
                .build();
        // Move forward to collect specimen from human player

        Action firstStage3 = drive.actionBuilder(new Pose2d(40,-40,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(5,-58), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // Return to border wall.
        // First sample stage.
        // Moves 1 - 3 consist of standard auto movement cycle.

// ------------------------------------------------------------------------------------------------------------------------------

        Action secondStage1 = drive.actionBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(40,-40), Math.toRadians(270))
                .waitSeconds(0.1)
                .build();
        // Move to loading zone, and turn around.

        Action secondStage2 = drive.actionBuilder(new Pose2d(40,-40, Math.toRadians(270)))
                .lineToY(-50)
                .waitSeconds(0.1)
                .build();
        // Move forward to collect specimen from human player.

        Action secondStage3 = drive.actionBuilder(new Pose2d(40,-40,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0,-58), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // return to border wall.
        // Second sample stage.

// --------------------------------------------------------------------------------------------------------------------------

        Action thirdStage1 = drive.actionBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(40,-40), Math.toRadians(270))
                .waitSeconds(0.1)
                .build();
        // Move to loading zone, and turn around.

        Action thirdStage2 = drive.actionBuilder(new Pose2d(40,-40, Math.toRadians(270)))
                .lineToY(-50)
                .waitSeconds(0.1)
                .build();
        // Move forward to collect specimen from human player.

        Action thirdStage3 = drive.actionBuilder(new Pose2d(40,-40,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-5,-58), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // return to border wall.
        // Third sample stage.
}



}
