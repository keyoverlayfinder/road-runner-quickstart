package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Competition;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
    public static class Slide {
        private final DcMotorEx rightSlide;
        private final DcMotorEx leftSlide;
        public Slide(HardwareMap hardwareMap, Telemetry telemetrySlide) {
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class SlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(0.8);
                    leftSlide.setPower(0.8);
                    initialized = true;
                }

                double rightSlideCurrentPosition = rightSlide.getCurrentPosition();
                double leftSlideCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("rightSlidePos", rightSlideCurrentPosition);
                packet.put("leftSlidePos", leftSlideCurrentPosition);
                if (rightSlideCurrentPosition < 3000.0 && leftSlideCurrentPosition < 3000) {
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

        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(-0.8);
                    leftSlide.setPower(-0.8);
                    initialized = true;
                }

                double slideRightCurrentPosition = rightSlide.getCurrentPosition();
                double slideLeftCurrentPosition = leftSlide.getCurrentPosition();
                packet.put("slideRightPos", slideRightCurrentPosition);
                packet.put("slideLeftPos", slideLeftCurrentPosition);
                if (slideRightCurrentPosition > 100.0 && slideLeftCurrentPosition > 100) {
                    return true;
                } else {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideDown(){
            return new SlideDown();
        }
    }

    // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // claw class
    public static class Claw {
        private final Servo rightClaw;
        private final Servo leftClaw;

        public Claw(HardwareMap hardwareMap, Telemetry telemetryClaw) {
            rightClaw = hardwareMap.get(Servo.class, "rightGripServo");
            leftClaw = hardwareMap.get(Servo.class,"leftGripServo");

            rightClaw.setDirection(Servo.Direction.FORWARD);
            leftClaw.setDirection(Servo.Direction.REVERSE);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(0.55);
                leftClaw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(1.0);
                leftClaw.setPosition(1.0);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

       // InitClaw makes the claw go as wide as possible so that we fit within 18 x 18 x 18 box when initialized, but avoids interference with normal game.
        public class InitClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightClaw.setPosition(2.0);
                leftClaw.setPosition(2);
                return false;
            }
        }
        public Action initClaw() {
            return new InitClaw();
        }
    }

    // ---------------------------------------------------------------------------------------------------------------------------------------------------
// setting up pivot motors
    public static class Pivot {
        private final DcMotorEx rightSlidePivot;
        private final DcMotorEx leftSlidePivot;
        public Pivot(HardwareMap hardwareMap, Telemetry telemetryPivot) {
            rightSlidePivot = hardwareMap.get(DcMotorEx.class, "rightSlidePivot");
            leftSlidePivot = hardwareMap.get(DcMotorEx.class,"leftSlidePivot");

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
                    rightSlidePivot.setPower(0.8);
                    leftSlidePivot.setPower(0.8);
                    initialized = true;
                }

                double rightPivotCurrentPosition = rightSlidePivot.getCurrentPosition();
                double leftPivotCurrentPosition = leftSlidePivot.getCurrentPosition();
                packet.put("rightPivotPos", rightPivotCurrentPosition);
                packet.put("leftPivotPos", leftPivotCurrentPosition);
                if (rightPivotCurrentPosition < 3000 && rightPivotCurrentPosition > 2000 && leftPivotCurrentPosition < 3000.0 && leftPivotCurrentPosition > 2000) {
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
    }

    // -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    private boolean pathFinished = false;
// Camera setup info
    OpenCvWebcam webcam1 = null;
    itdBlobPipeline itdCam = new itdBlobPipeline(); // Refers to itdBlobPipeline for camera info

    int width = 1280, height = 720;

    // -------------------------------------------------------------------------------------------------------------------------------------------------

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

        Action firstStage1 = drive.actionBuilder(new Pose2d(25, -60, Math.toRadians(90)))
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

        // ----------------------------------------------------------------------------------------------------------------------------

        Actions.runBlocking(claw.initClaw());
        Actions.runBlocking(pivot.initPivot());
        Actions.runBlocking(slide.slideDown());



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
            }});

        waitForStart();

        while (opModeIsActive()) {
            // Add any logic or telemetry here to monitor pipeline results
            telemetry.addData("Frame Count", webcam1.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US,"%.2f", webcam1.getFps()));
            telemetry.update();

            // Sets movement logic into action
            if (!pathFinished) {
                Actions.runBlocking(firstStage1);
                sleep(200);
                Actions.runBlocking(firstStage2);
                sleep(200);
                Actions.runBlocking(firstStage3);
                sleep(200);
                pathFinished = true;
            } else {
                webcam1.stopStreaming();
                return;
            }
        }

    }
}
