package org.firstinspires.ftc.teamcode.RealOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@Autonomous
public class RedAudienceAutoPark extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.xRailRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.xRailRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.xRailExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.xRailExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double xRailRotMin = drive.xRailRot.getCurrentPosition();

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
                .lineTo(new Vector2d(-29, 0))
                .build();
        Trajectory trajRight0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, -Math.toRadians(90)))
                .lineTo(new Vector2d(-27, -1)) //placeholder
                .build();
        Trajectory trajLeft0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(-26.5, 3)) //placeholder
                .build();

        drive.followTrajectory(traj1);
        double backSense = drive.Sense(drive.colorBack);
        double leftSense = drive.Sense(drive.colorLeft);
        drive.hangerServo.setPosition(0);
        if (backSense < 2.9) { //if team object is at the BACK
            telemetry.addData("Back", backSense);
            telemetry.update();
            drive.followTrajectory(trajBack);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(trajBack0);
            drive.grabberServoFront(1);
            drive.setXrailPower(-.5,0);
            sleep(1000);
            drive.setXrailPower(0,0);
        } else if (leftSense <2.9) { //if team object is on the LEFT
            telemetry.addData("Left", leftSense);
            telemetry.update();
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(trajLeft0);
            drive.grabberServoFront(1);
            drive.setXrailPower(-.5,0);
            sleep(1000);
            drive.setXrailPower(0,0);

        } else if (leftSense>=2.9 && backSense >= 2.9) { //if team object is on the RIGHT
            telemetry.addData("Right", leftSense);
            telemetry.update();
            //drive.followTrajectory(trajBack);
            drive.turn(-Math.toRadians(90));
            drive.followTrajectory(trajRight0);
            drive.grabberServoFront(1);
            drive.setXrailPower(-.5,0);
            sleep(1000);
            drive.setXrailPower(0,0);
        } else { //if like the sun explodes idk
            telemetry.addData("?????", leftSense);
        }
        telemetry.update();


    }
}
