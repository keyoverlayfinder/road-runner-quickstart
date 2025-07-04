package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Competition;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionIntoTheDeep.itdBlobPipeline;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;

@Config
@Autonomous(name="BlueAuto", group="competition")
public class specimenOpMode extends LinearOpMode {

    // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // lift class
    public class Slide {
        private final DcMotorEx rightSlide;
        private final DcMotorEx leftSlide;

        public Slide(HardwareMap hardwareMap, Telemetry telemetrySlide) {
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class SlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                if (!initialized) {
                    initialized = true;
                }

                double rightSlideCurrentPosition = rightSlide.getCurrentPosition();
                double leftSlideCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("rightSlidePos", rightSlideCurrentPosition);
                packet.put("leftSlidePos", leftSlideCurrentPosition);
                if (rightSlideCurrentPosition > -1400) {
                    rightSlide.setPower(0.8);
                    leftSlide.setPower(-0.8);
                    return true;
                } else {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                    return false;
                }

            }
        }

        public Action slideUp() {
            return new SlideUp();
        }

        public class GroundSlide implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                if (!initialized) {
                    initialized = true;
                }

                double slideRightCurrentPosition = rightSlide.getCurrentPosition();
                double slideLeftCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("slideRightPos", slideRightCurrentPosition);
                packet.put("slideLeftPos", slideLeftCurrentPosition);
                if (slideRightCurrentPosition > -1200) {
                    rightSlide.setPower(0.8);
                    leftSlide.setPower(-0.8);
                    return true;
                } else {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action groundSlide() {
            return new GroundSlide();
        }

        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                if (!initialized) {
                    initialized = true;
                }

                double slideRightCurrentPosition = rightSlide.getCurrentPosition();
                double slideLeftCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("slideRightPos", slideRightCurrentPosition);
                packet.put("slideLeftPos", slideLeftCurrentPosition);
                if (slideRightCurrentPosition > -100.0) {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                    return true;
                } else {
                    rightSlide.setPower(0.8);
                    leftSlide.setPower(-0.8);
                    return false;
                }
            }
        }

        public Action slideDown() {
            return new SlideDown();
        }
    }

    // ---------------------------------------------------------------------------------------------------------------------------------------------------
// setting up pivot motors
    public class Pivot {
        private final DcMotorEx rightSlidePivot;
        private final DcMotorEx leftSlidePivot;

        public Pivot(HardwareMap hardwareMap, Telemetry telemetryPivot) {
            rightSlidePivot = hardwareMap.get(DcMotorEx.class, "rightSlidePivot");
            leftSlidePivot = hardwareMap.get(DcMotorEx.class, "leftSlidePivot");

            rightSlidePivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlidePivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            leftSlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightSlidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlidePivot.setDirection(DcMotorSimple.Direction.FORWARD);
            leftSlidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class InitPivot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition > -500) {
                    rightSlidePivot.setPower(0.8);
                    leftSlidePivot.setPower(-0.8);
                    return true;
                } else {
                    rightSlidePivot.setPower(0);
                    leftSlidePivot.setPower(0);
                    return false;
                }
            }
        }

        public Action initPivot() {
            return new InitPivot();
        }

        public class GroundPivot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition > -6000) {
                    rightSlidePivot.setPower(0.8);
                    leftSlidePivot.setPower(-0.8);
                    return true;
                } else {
                    rightSlidePivot.setPower(0);
                    leftSlidePivot.setPower(0);
                    return false;
                }
            }
        }

        public Action groundPivot() {
            return new GroundPivot();
        }


        public class SpecimenPivot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition > -3500) {
                    rightSlidePivot.setPower(0.8);
                    leftSlidePivot.setPower(-0.8);
                    return true;
                } else {
                    rightSlidePivot.setPower(0);
                    leftSlidePivot.setPower(0);
                    return false;
                }
            }
        }

        public Action specimenPivot() {
            return new SpecimenPivot();
        }
    }

    // claw class
    public class Claw {
        private final Servo rightClaw;
        private final Servo leftClaw;

        public Claw(HardwareMap hardwareMap, Telemetry telemetryClaw) {
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");

            rightClaw.setDirection(Servo.Direction.FORWARD);
            leftClaw.setDirection(Servo.Direction.REVERSE);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                rightClaw.setPosition(0.25);
                leftClaw.setPosition(0.25);
                return false;
            }
        }

        public Action closeClaw() {
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                rightClaw.setPosition(.75);
                leftClaw.setPosition(.75);
                return true;
            }
        }

        public Action openClaw() {
            return new Claw.OpenClaw();
        }

        // InitClaw makes the claw go as wide as possible so that we fit within 18 x 18 x 18 box when initialized, but avoids interference with normal game.
        public class InitClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!opModeIsActive()) return false;

                rightClaw.setPosition(.8);
                leftClaw.setPosition(.8);
                return false;
            }
        }

        public Action initClaw() {
            return new InitClaw();
        }
    }

    // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        //Set up the claw
        Claw claw = new specimenOpMode.Claw(hardwareMap, telemetry);

        //Set up the slides
        Slide slide = new Slide(hardwareMap, telemetry);

        //Set up the Pivots
        Pivot pivot = new specimenOpMode.Pivot(hardwareMap, telemetry);


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -60, Math.toRadians(90)));

        Action firstStage1 = drive.actionBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                .strafeTo(new Vector2d(55, -60))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(40, -60))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(270))
                .waitSeconds(0.1)
                .build();
        // Move to loading zone, and turn around

        Action firstStage2 = drive.actionBuilder(new Pose2d(40, -40, Math.toRadians(270)))
                .lineToY(-50)
                .waitSeconds(0.1)
                .build();
        // Move forward to collect specimen from human player

        Action firstStage3 = drive.actionBuilder(new Pose2d(40, -40, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(5, -40), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // Return to border wall.
        // First specimen stage.
        // Moves 1 - 3 consist of standard auto movement cycle.

// ------------------------------------------------------------------------------------------------------------------------------

        Action secondStage1 = drive.actionBuilder(new Pose2d(5, -40, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(270))
                .waitSeconds(0.1)
                .build();
        // Move to loading zone, and turn around.

        Action secondStage2 = drive.actionBuilder(new Pose2d(40, -40, Math.toRadians(270)))
                .lineToY(-50)
                .waitSeconds(0.1)
                .build();
        // Move forward to collect specimen from human player.

        Action secondStage3 = drive.actionBuilder(new Pose2d(40, -40, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // return to border wall.
        // Second specimen stage.

// --------------------------------------------------------------------------------------------------------------------------

        Action thirdStage1 = drive.actionBuilder(new Pose2d(0, -40, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(270))
                .waitSeconds(0.1)
                .build();
        // Move to loading zone, and turn around.

        Action thirdStage2 = drive.actionBuilder(new Pose2d(40, -40, Math.toRadians(270)))
                .lineToY(-50)
                .waitSeconds(0.1)
                .build();
        // Move forward to collect specimen from human player.

        Action thirdStage3 = drive.actionBuilder(new Pose2d(40, -40, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-5, -40), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // return to border wall.
        // Third specimen stage.

        Action park = drive.actionBuilder(new Pose2d(-5, -50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(56, -58), Math.toRadians(180))
                .build();

        // ----------------------------------------------------------------------------------------------------------------------------

        Actions.runBlocking(claw.initClaw());
        Actions.runBlocking(pivot.initPivot());

        waitForStart();
        while (opModeIsActive()) {
            // Sets movement logic into action
            Actions.runBlocking(
                    new SequentialAction(
                            firstStage1,
                            claw.openClaw(),
                            pivot.groundPivot(),
                            slide.groundSlide(),
                            firstStage2,
                            claw.closeClaw(),
                            pivot.initPivot(),
                            firstStage3,
                            pivot.specimenPivot(),
                            slide.slideUp(),
                            claw.initClaw(),
                            slide.slideDown(),
                            pivot.initPivot()
                    )
            );
            Actions.runBlocking(
                        new SequentialAction(
                                secondStage1,
                                claw.openClaw(),
                                pivot.groundPivot(),
                                slide.groundSlide(),
                                secondStage2,
                                claw.closeClaw(),
                                pivot.initPivot(),
                                secondStage3,
                                pivot.specimenPivot(),
                                slide.slideUp(),
                                claw.initClaw(),
                                slide.slideDown(),
                                pivot.initPivot()
                        )
            );
            Actions.runBlocking(
                        new SequentialAction(
                                thirdStage1,
                                claw.openClaw(),
                                pivot.groundPivot(),
                                slide.groundSlide(),
                                thirdStage2,
                                claw.closeClaw(),
                                pivot.initPivot(),
                                thirdStage3,
                                pivot.specimenPivot(),
                                slide.slideUp(),
                                claw.initClaw(),
                                slide.slideDown(),
                                pivot.initPivot(),
                                park
                            )
            );

//            Actions.runBlocking(
//                    new SequentialAction(
//                            secondStage1,
//                            claw.openClaw(),
//                            pivot.groundPivot(),
//                            slide.groundSlide(),
//                            secondStage2,
//                            claw.closeClaw(),
//                            pivot.initPivot(),
//                            secondStage3,
//                            pivot.specimenPivot(),
//                            slide.slideUp(),
//                            claw.initClaw(),
//                            slide.slideDown(),
//                            pivot.initPivot()
//                    )
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            thirdStage1,
//                            claw.openClaw(),
//                            pivot.groundPivot(),
//                            slide.groundSlide(),
//                            thirdStage2,
//                            claw.closeClaw(),
//                            pivot.initPivot(),
//                            thirdStage3,
//                            pivot.specimenPivot(),
//                            slide.slideUp(),
//                            claw.initClaw(),
//                            slide.slideDown(),
//                            pivot.initPivot(),
//                            park
//                    )
//            );
        }
    }
}

