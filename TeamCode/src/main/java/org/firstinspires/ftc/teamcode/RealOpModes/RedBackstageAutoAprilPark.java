package org.firstinspires.ftc.teamcode.RealOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class RedBackstageAutoAprilPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.xRailRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.xRailRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double xRailRotMin = drive.xRailRot.getCurrentPosition();

        double DESIRED_DISTANCE = 19.0; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
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
        drive.initAprilTag(aprilTag, visionPortal);


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
                .lineTo(new Vector2d(-31, 0))
                .build();
        Trajectory trajBack1 = drive.trajectoryBuilder(trajBack0.end())
                .strafeTo(new Vector2d(-20, 0))
                .build();
        Trajectory trajBack2 = drive.trajectoryBuilder(trajBack1.end())
                .strafeTo(new Vector2d(-5, 40))
                .build();

        Trajectory trajRight0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, -Math.toRadians(90)))
                .lineTo(new Vector2d(-29.5, -3.5)) //placeholder
                .build();
        Trajectory trajRight1 = drive.trajectoryBuilder(trajRight0.end())
                .strafeTo(new Vector2d(-25, 6))
                .build();
        Trajectory trajRightApril = drive.trajectoryBuilder(trajRight1.end())
                .strafeTo(new Vector2d(-20, 10))
                .build();
        Trajectory trajRight2 = drive.trajectoryBuilder(trajRightApril.end(), Math.toRadians(180))
                .strafeTo(new Vector2d(-5, 40))
                .build();

        Trajectory trajLeft0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(-29.5, 3.7)) //placeholder
                .build();
        Trajectory trajLeft1 = drive.trajectoryBuilder(trajLeft0.end())
                .strafeTo(new Vector2d(-25, 0))
                .build();
        Trajectory trajLeftApril = drive.trajectoryBuilder(trajLeft1.end())
                .strafeTo(new Vector2d(-20, 10))
                .build();
        Trajectory trajLeft2 = drive.trajectoryBuilder(trajLeftApril.end())
                .strafeTo(new Vector2d(-5, 40))
                .build();

        drive.followTrajectory(traj1);
        double backSense = drive.Sense(drive.colorBack);
        double leftSense = drive.Sense(drive.colorLeft);
        if (backSense < 2.9) { //if team object is at the BACK
            telemetry.addData("Back", backSense);
            telemetry.update();
            drive.followTrajectory(trajBack);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(trajBack0);
            drive.grabberServo(1);
            drive.setXrailPower(-.5,0);
            sleep(1000);
            drive.setXrailPower(0,0);
            drive.followTrajectory(trajBack1);
            drive.followTrajectory(trajBack2);
            /*drive.turn(-Math.toRadians(90));
            drive.followTrajectory(trajBack1);
            telemetry.addData("Back park", backSense);
            telemetry.update();
            sleep(1000);
            while(drive.xRailRot.getCurrentPosition() > xRailRotMin) {
                drive.setXrailPower(.5, 0);
            }*/
        } else if (leftSense <2.9) { //if team object is on the RIGHT
            telemetry.addData("Left", leftSense);
            telemetry.update();
            DESIRED_TAG_ID = 6;
            //drive.followTrajectory(trajBack);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(trajLeft0);
            //drive.grabberServo(1);
            //drive.setXrailPower(-.5,0);
            //sleep(1000);
            //drive.setXrailPower(0,0);
            drive.followTrajectory(trajLeft1);
            drive.followTrajectory(trajLeftApril);
            while (done = false) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                }

                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                if (targetFound) {

                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;
                    if (rangeError < DESIRED_DISTANCE + .1) {
                        done = true;
                    }

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    power  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    power  = 0;  // Reduce drive rate to 50%.
                    strafe = 0;  // Reduce strafe rate to 50%.
                    turn   = 0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                drive.moveRobot(power, strafe, turn);
                sleep(10);


            }
            drive.followTrajectory(trajLeft2);

        } else if (leftSense>=2.9 && backSense >= 2.9) { //if team object is on the LEFT
            telemetry.addData("Right", leftSense);
            telemetry.update();
            DESIRED_TAG_ID = 4;
            //drive.followTrajectory(trajBack);
            drive.turn(-Math.toRadians(90));
            drive.followTrajectory(trajRight0);
            //drive.grabberServo(1);
            //drive.setXrailPower(-.5,0);
            //sleep(1000);
            //drive.setXrailPower(0,0);
            drive.followTrajectory(trajRight1);
            drive.followTrajectory(trajRightApril);
            drive.turn(Math.toRadians(180));
            while (done = false) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                }

                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                if (targetFound) {

                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;
                    if (rangeError < DESIRED_DISTANCE + .1) {
                        done = true;
                    }

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    power  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    power  = 0;  // Reduce drive rate to 50%.
                    strafe = 0;  // Reduce strafe rate to 50%.
                    turn   = 0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                drive.moveRobot(power, strafe, turn);
                sleep(10);


            }
            drive.followTrajectory(trajRight2);

        } else { //if like the sun explodes idk
            telemetry.addData("?????", leftSense);
        }
        telemetry.update();
        sleep(10000);


    }

}
