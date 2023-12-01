package org.firstinspires.ftc.teamcode.RealOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class RedBackstageAutoAprilParkUpdated extends LinearOpMode {


    double DESIRED_DISTANCE = 15.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    double MAX_AUTO_SPEED = 0.8;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    //set this for each part of the opmode
    int DESIRED_TAG_ID = 6;     // Choose the tag you want to approach or set to -1 for ANY tag.
    VisionPortal visionPortal = null;               // Used to manage the video source.
    AprilTagProcessor aprilTag = null;              // Used for managing the AprilTag detection process.
    AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  power           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)
    boolean done = false;
    double i = 0;
    Pose2d postPose;




    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.xRailRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.xRailRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double xRailRotMin = drive.xRailRot.getCurrentPosition();
        ElapsedTime elapsedRunTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(drive.Webcam1)
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, 250, visionPortal);

        waitForStart();
        if (isStopRequested()) return;



        //drive.setPoseEstimate();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-26.5, 0))
                .build();
        Trajectory trajBack =  drive.trajectoryBuilder(traj1.end(), true)
                .lineTo(new Vector2d(-25, 0))
                .build();
        Trajectory trajBack0 = drive.trajectoryBuilder(new Pose2d(-25, 0, Math.toRadians(180)))
                .lineTo(new Vector2d(-31.5, 0))
                .build();
        Trajectory trajBack1 = drive.trajectoryBuilder(trajBack0.end())
                .strafeTo(new Vector2d(-20, 0))
                .build();

        Trajectory trajRight0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, -Math.toRadians(90)))
                .lineTo(new Vector2d(-29.5, -3.7)) //placeholder
                .build();
        Trajectory trajRight1 = drive.trajectoryBuilder(trajRight0.end())
                .strafeTo(new Vector2d(-28, 6))
                .build();

        Trajectory trajLeft0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(-29.5, 3.7)) //placeholder
                .build();
        Trajectory trajLeft1 = drive.trajectoryBuilder(trajLeft0.end())
                .strafeTo(new Vector2d(-25, 0))
                .build(); //this is where the robot starts to scan apriltag

        drive.followTrajectory(traj1);
        double backSense = drive.Sense(drive.colorBack);
        double leftSense = drive.Sense(drive.colorLeft);
        drive.hangerServo.setPosition(0);
        if (backSense < 2.9) { //if team object is at the BACK
            telemetry.addData("Back", backSense);
            DESIRED_TAG_ID = 5;
            telemetry.update();
            drive.hangerServo.setPosition(0);
            drive.followTrajectory(trajBack);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(trajBack0);
            drive.grabberServoFront(1);
            drive.setXrailPower(-1,0);
            sleep(700);
            drive.setXrailPower(0,0);
            drive.followTrajectory(trajBack1);
            drive.turn(-Math.toRadians(90));
            elapsedRunTime.reset();
            telemetry.addData("Time: ", elapsedRunTime.time(TimeUnit.SECONDS));
            AprilRun(DESIRED_TAG_ID, elapsedRunTime, drive);
            drive.updatePoseEstimate();
            postPose = drive.getPoseEstimate();
            i = postPose.getY();
            Trajectory trajBackAdjust = drive.trajectoryBuilder(postPose, Math.toRadians(180))
                    .strafeTo(new Vector2d(-23, i+11))
                    .build();
            Trajectory trajBack2 = drive.trajectoryBuilder(trajBackAdjust.end())
                    .strafeTo(new Vector2d(-5, i)) //check this
                    .build();
            telemetry.addData("Position i: ", drive.getPoseEstimate());
            telemetry.update();
            sleep(1000); //temporary
            drive.followTrajectory(trajBackAdjust);
            telemetry.addData("Position e: ", drive.getPoseEstimate());
            telemetry.update();
            placeYellow(drive);
            //drive.followTrajectory(trajBack2);

        } else if (leftSense <2.9) { //if team object is on the RIGHT
            telemetry.addData("Left", leftSense);
            telemetry.update();
            DESIRED_TAG_ID = 6;
            drive.hangerServo.setPosition(0);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(trajLeft0);
            drive.grabberServoFront(1);
            drive.setXrailPower(-1,0);
            sleep(700);
            drive.setXrailPower(0,0);
            drive.followTrajectory(trajLeft1);
            elapsedRunTime.reset();
            telemetry.addData("Time: ", elapsedRunTime.time(TimeUnit.SECONDS));
            AprilRun(DESIRED_TAG_ID, elapsedRunTime, drive);
            drive.updatePoseEstimate();
            postPose = drive.getPoseEstimate();
            i = postPose.getY();
            Trajectory trajLeftAdjust = drive.trajectoryBuilder(postPose, Math.toRadians(90))
                    .strafeTo(new Vector2d(-20, i+11))
                    .build();
            Trajectory trajLeft2 = drive.trajectoryBuilder(trajLeftAdjust.end())
                    .strafeTo(new Vector2d(-5, i))
                    .build();
            telemetry.addData("Position i: ", drive.getPoseEstimate());
            telemetry.update();
            sleep(1000);
            drive.followTrajectory(trajLeftAdjust);
            telemetry.addData("Position e: ", drive.getPoseEstimate());
            telemetry.update();
            placeYellow(drive);
            //drive.followTrajectory(trajLeft2);

        } else if (leftSense>=2.9 && backSense >= 2.9) { //if team object is on the LEFT
            telemetry.addData("Right", leftSense);
            telemetry.update();
            DESIRED_TAG_ID = 4;
            drive.hangerServo.setPosition(0);
            //drive.followTrajectory(trajBack);
            drive.turn(-Math.toRadians(90));
            drive.followTrajectory(trajRight0);
            drive.grabberServoFront(1);
            drive.setXrailPower(-1,0);
            sleep(700);
            drive.setXrailPower(0,0);
            drive.followTrajectory(trajRight1);
            drive.turn(Math.toRadians(180));
            elapsedRunTime.reset();
            telemetry.addData("Time: ", elapsedRunTime.time(TimeUnit.SECONDS));
            AprilRun(DESIRED_TAG_ID, elapsedRunTime, drive);
            drive.updatePoseEstimate();
            postPose = drive.getPoseEstimate();
            i = postPose.getY();
            Trajectory trajRightAdjust = drive.trajectoryBuilder(postPose, -Math.toRadians(90))
                    .strafeTo(new Vector2d(-32, i+11))
                    .build();
            Trajectory trajRight2 = drive.trajectoryBuilder(trajRightAdjust.end())
                    .strafeTo(new Vector2d(-5, i))
                    .build();
            telemetry.addData("Position i: ", drive.getPoseEstimate());
            telemetry.update();
            sleep(1000);
            drive.followTrajectory(trajRightAdjust);
            telemetry.addData("Position e: ", drive.getPoseEstimate());
            telemetry.update();
            placeYellow(drive);
            //drive.followTrajectory(trajRight2);

        } else { //if like the sun explodes idk
            telemetry.addData("?????", leftSense);
        }
        telemetry.update();
        sleep(3000);


    }

    private void AprilRun(int DESIRED_TAG_ID, ElapsedTime elapsedRunTime, SampleMecanumDrive drive) {
        while (done == false && opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;
            //this next line has problems
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("line: ", 156);
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        telemetry.addData("Tagid: ", detection.id);
                        telemetry.update();
                        // don't look any further.
                        break;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            telemetry.update();
            //telemetry.addData("here: ", targetFound);
            //telemetry.addData("desiredTag: ", desiredTag.id);

            // Tell the driver what we see, and what to do.
            if (desiredTag != null) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }
            telemetry.update();
            //Drive to target Automatically
            double rangeError = 50;
            if (targetFound) {


                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                power  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            } else {
                // don't
                power  = 0;  // Reduce drive rate to 50%.
                strafe = 0;  // Reduce strafe rate to 50%.
                turn   = 0;  // Reduce turn rate to 33%.
            }
            if (rangeError < DESIRED_DISTANCE + .1 && elapsedRunTime.time(TimeUnit.SECONDS) > 3) {
                done = true;
            }
            telemetry.update();
            telemetry.addData("here: line ", 286);



            // Apply desired axes motions to the drivetrain.
            drive.moveRobot(power, strafe, turn);


        }

    }

    private void setManualExposure(int exposureMS, int gain, VisionPortal visionPortal) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void placeYellow (SampleMecanumDrive drive) {
        drive.setXrailPower(-1, 0);
        sleep(800);
        drive.setXrailPower(-1, .7);
        sleep(1200);
        drive.setXrailPower(0, 1);
        sleep(1700);
        drive.setXrailPower(0, 0);
        drive.grabberServoBack(1);
        sleep(200); //removable
        drive.setXrailPower(0, -1);
        sleep(200);
        drive.setXrailPower(-.3, -1);
        sleep(500);
        drive.setXrailPower(0, 0);
    }

}
