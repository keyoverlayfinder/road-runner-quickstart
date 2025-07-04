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
@Autonomous(name="bucketAuto", group="competition")
public class sampleOpMode extends LinearOpMode {
    //constants for simple correction
    final Pose2d bucketDropOffPose = new Pose2d(-51,-51, Math.toRadians(45));
    final double sampleOffset = -45;

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
                if (!initialized) {
                    initialized = true;
                }

                double rightSlideCurrentPosition = rightSlide.getCurrentPosition();
                double leftSlideCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("rightSlidePos", rightSlideCurrentPosition);
                packet.put("leftSlidePos", leftSlideCurrentPosition);
                if (rightSlideCurrentPosition > -2400) {
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
                if (!initialized) {
                    initialized = true;
                }

                double slideRightCurrentPosition = rightSlide.getCurrentPosition();
                double slideLeftCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("slideRightPos", slideRightCurrentPosition);
                packet.put("slideLeftPos", slideLeftCurrentPosition);
                if (slideRightCurrentPosition > -1600) {
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
                if (!initialized) {
                    initialized = true;
                }

                double slideRightCurrentPosition = rightSlide.getCurrentPosition();
                double slideLeftCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("slideRightPos", slideRightCurrentPosition);
                packet.put("slideLeftPos", slideLeftCurrentPosition);
                if (slideRightCurrentPosition < -100.0) {
                    rightSlide.setPower(-0.8);
                    leftSlide.setPower(0.8);
                    return true;
                } else {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
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
    public static class Pivot {
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
                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition > -5750) {
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


        public class BucketPivot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition < -2200) {
                    rightSlidePivot.setPower(-0.8);
                    leftSlidePivot.setPower(0.8);
                    return true;
                } else {
                    rightSlidePivot.setPower(0);
                    leftSlidePivot.setPower(0);
                    return false;
                }
            }
        }
        public Action bucketPivot() {
            return new BucketPivot();
        }

        public class AscentPivot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition > -2000) {
                    rightSlidePivot.setPower(-0.8);
                    leftSlidePivot.setPower(0.8);
                    return true;
                } else {
                    rightSlidePivot.setPower(0);
                    leftSlidePivot.setPower(0);
                    return false;
                }
            }
        }
        public Action ascentPivot() {
            return new AscentPivot();
        }


        public class UpPivotIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition < -2600) {
                    rightSlidePivot.setPower(-0.8);
                    leftSlidePivot.setPower(0.8);
                    return true;
                } else {
                    rightSlidePivot.setPower(0);
                    leftSlidePivot.setPower(0);
                    return false;
                }
            }
        }
        public Action upPivotIn() {
            return new UpPivotIn();
        }

        public class UpPivotOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                double rightPivotAbsolutePosition = (rightSlidePivot.getCurrentPosition());
                double leftPivotAbsolutePosition = (leftSlidePivot.getCurrentPosition());
                packet.put("rightPivotPos", rightPivotAbsolutePosition);
                packet.put("leftPivotPos", leftPivotAbsolutePosition);

                if (rightPivotAbsolutePosition > -2600) {
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

        public Action upPivotOut(){
            return new UpPivotOut();
        }
    }

    // claw class
    public static class Claw {
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
                rightClaw.setPosition(0.25);
                leftClaw.setPosition(0.25);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(.75);
                leftClaw.setPosition(.75);
                return true;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        // InitClaw makes the claw go as wide as possible so that we fit within 18 x 18 x 18 box when initialized, but avoids interference with normal game.
        public class InitClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(.25);
                leftClaw.setPosition(.25);
                return false;
            }
        }

        public Action initClaw() {
            return new InitClaw();
        }
    }

    // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    private boolean pathFinished = false;
    OpenCvWebcam webcam1 = null;
    itdBlobPipeline itdCam = new itdBlobPipeline();

    int width = 1280, height = 720;

    @Override
    public void runOpMode() throws InterruptedException {

        //Set up the claw
        Claw claw = new Claw(hardwareMap, telemetry);

        //Set up the slides
        Slide slide = new Slide(hardwareMap, telemetry);

        //Set up the Pivots
        Pivot pivot = new Pivot(hardwareMap, telemetry);

        // Set up the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // The view ID is needed for displaying the camera feed in the FTC app
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Initialize the webcam
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Set the camera's pipeline to the one you've created
        webcam1.setPipeline(itdCam);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -60, Math.toRadians(90)));

        Action firstStage1 = drive.actionBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                .strafeToLinearHeading(bucketDropOffPose.position, bucketDropOffPose.heading)
                .waitSeconds(0.1)
                .build();
        // Move to bucket to deposit initial sample
        // First specimen stage

// ------------------------------------------------------------------------------------------------------------------------------

        Action secondStage1 = drive.actionBuilder(bucketDropOffPose)
                .strafeToLinearHeading(new Vector2d(-47.5, sampleOffset), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // Move from bucket to first ground sample

        Action secondStage2 = drive.actionBuilder(new Pose2d(-47.5, sampleOffset, Math.toRadians(90)))
                .strafeToLinearHeading(bucketDropOffPose.position, bucketDropOffPose.heading)
                .waitSeconds(0.1)
                .build();
        // Move to bucket

// --------------------------------------------------------------------------------------------------------------------------

        Action thirdStage1 = drive.actionBuilder(bucketDropOffPose)
                .strafeToLinearHeading(new Vector2d(-58, sampleOffset), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // Move from bucket to second ground sample

        Action thirdStage2 = drive.actionBuilder(new Pose2d(-58, sampleOffset, Math.toRadians(90)))
                .strafeToLinearHeading(bucketDropOffPose.position,bucketDropOffPose.heading)
                .waitSeconds(0.1)
                .build();
        // Move to bucket

// ----------------------------------------------------------------------------------------------------------------------------

        Action fourthStage1 = drive.actionBuilder(bucketDropOffPose)
                .strafeToLinearHeading(new Vector2d((-97 - sampleOffset),-25), Math.toRadians(180))
                .waitSeconds(0.1)
                .build();
        //Move to third ground sample
        Action fourthStage2 = drive.actionBuilder(new Pose2d(-52,-25,Math.toRadians(180)))
                .strafeToSplineHeading(bucketDropOffPose.position,bucketDropOffPose.heading)
                .build();
        //Move to bucket
        Action fourthStage3 = drive.actionBuilder(bucketDropOffPose)
                .strafeToLinearHeading(new Vector2d(bucketDropOffPose.position.x,-12),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-24,-12),Math.toRadians(180))
                .build();
        //Move to observation zone for ascent 1


        Actions.runBlocking(claw.initClaw());
        Actions.runBlocking(pivot.initPivot());

        // Open the camera asynchronously to avoid blocking the thread
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming in MJPEG format with a specified resolution
                webcam1.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_RIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened: " + errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Add any logic or telemetry here to monitor pipeline results
            telemetry.addData("Frame Count", webcam1.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam1.getFps()));
            telemetry.update();

            // Sets movement logic into action
            if (opModeIsActive()) {
                //initial sample
                Actions.runBlocking(
                        new SequentialAction(
                                firstStage1,
                                pivot.upPivotIn(),
                                slide.slideUp(),
                                pivot.bucketPivot(),
                                claw.openClaw(),
                                pivot.upPivotOut(),
                                slide.slideDown()
                        )
                );

                //first ground sample
                Actions.runBlocking(
                        new SequentialAction(
                                secondStage1,
                                pivot.groundPivot(),
                                slide.groundSlide(),
                                claw.closeClaw(),
                                slide.slideDown(),
                                pivot.upPivotIn(),
                                secondStage2,
                                slide.slideUp(),
                                pivot.bucketPivot(),
                                claw.openClaw(),
                                pivot.upPivotOut(),
                                slide.slideDown()
                        )
                );
                //seconds ground sample
                Actions.runBlocking(
                        new SequentialAction(
                                thirdStage1,
                                pivot.groundPivot(),
                                slide.groundSlide(),
                                claw.closeClaw(),
                                slide.slideDown(),
                                pivot.upPivotIn(),
                                thirdStage2,
                                slide.slideUp(),
                                pivot.bucketPivot(),
                                claw.openClaw(),
                                pivot.upPivotOut(),
                                slide.slideDown()
                        )
                );
                //third ground sample and park
                Actions.runBlocking(
                        new SequentialAction(
                                fourthStage1,
                                pivot.groundPivot(),
                                slide.groundSlide(),
                                claw.closeClaw(),
                                slide.slideDown(),
                                pivot.upPivotIn(),
                                fourthStage2,
                                slide.slideUp(),
                                pivot.bucketPivot(),
                                claw.openClaw(),
                                pivot.upPivotOut(),
                                slide.slideDown(),
                                fourthStage3,
                                slide.slideUp(),
                                pivot.ascentPivot()
                        )
                );
            }
        }
    }
}