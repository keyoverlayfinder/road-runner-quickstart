package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Competition;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionIntoTheDeep.itdBlobPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="BlueAuto", group="competition")
public class specimenOpMode extends LinearOpMode {
    private boolean pathFinished = false;

    OpenCvWebcam webcam1 = null;
    itdBlobPipeline itdCam = new itdBlobPipeline();

    int width = 1280, height = 720;

    @Override
    public void runOpMode() throws InterruptedException {
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
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap, telemetry);
        Pivot pivot = new Pivot(hardwareMap, telemetry);

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
                .strafeToLinearHeading(new Vector2d(5, -58), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // Return to border wall.
        // First specimen stage.
        // Moves 1 - 3 consist of standard auto movement cycle.

// ------------------------------------------------------------------------------------------------------------------------------

        Action secondStage1 = drive.actionBuilder(new Pose2d(5, -58, Math.toRadians(90)))
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
                .strafeToLinearHeading(new Vector2d(0, -58), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // return to border wall.
        // Second specimen stage.

// --------------------------------------------------------------------------------------------------------------------------

        Action thirdStage1 = drive.actionBuilder(new Pose2d(0, -58, Math.toRadians(90)))
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
                .strafeToLinearHeading(new Vector2d(-5, -58), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();
        // return to border wall.
        // Third specimen stage.

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
            telemetry.addData("FPS", String.format("%.2f", webcam1.getFps()));
            telemetry.update();

            // Sets movement logic into action
            if (!pathFinished) {
                Actions.runBlocking(firstStage1);
                Actions.runBlocking(firstStage2);
                Actions.runBlocking(firstStage3);
                pathFinished = true;
            } else {
                webcam1.stopStreaming();
                return;
            }
        }

    }
    public class HangSpecimen implements Action {
        private final Slide slide;
        private final Claw claw;
        private final Pivot pivot;
        private final int slidePosition;
        private final double slidePower = 0.5;

        public HangSpecimen(Slide slide, Claw claw, Pivot pivot, int slidePosition) {
            this.slide = slide;
            this.claw = claw;
            this.pivot = pivot;
            this.slidePosition = slidePosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.pivotToPosition(-300, 0.75, 5);
            slide.moveToPosition(700, 0.75);
            return false;
        }
    }
    public Action HangSpecimen(Slide slide, Claw claw, Pivot pivot, int slidePosition) {
        return new specimenOpMode().HangSpecimen(slide, claw, pivot, slidePosition);
    }
}
