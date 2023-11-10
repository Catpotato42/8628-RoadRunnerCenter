package org.firstinspires.ftc.teamcode.RealOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class BlueBackstageAutoPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.xRailRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.xRailRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double xRailRotMin = drive.xRailRot.getCurrentPosition();

        waitForStart();
        if (isStopRequested()) return;

        //drive.setPoseEstimate();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-29.5, 0))
                .build();

        Trajectory trajBack =  drive.trajectoryBuilder(traj1.end(), true)
                .lineTo(new Vector2d(-25, 0))
                .build();
        Trajectory trajBack0 = drive.trajectoryBuilder(new Pose2d(-25, 0, Math.toRadians(180)))
                .lineTo(new Vector2d(-30, 0))
                .build();
        Trajectory trajBack1 = drive.trajectoryBuilder(trajBack0.end())
                .strafeTo(new Vector2d(-20, 0))
                .build();
        Trajectory trajBack2 = drive.trajectoryBuilder(trajBack1.end())
                .strafeTo(new Vector2d(-5, -40))
                .build();

        Trajectory trajRight0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, -Math.toRadians(90)))
                .lineTo(new Vector2d(-29.5, -3.5)) //placeholder
                .build();
        Trajectory trajRight1 = drive.trajectoryBuilder(trajRight0.end())
                .strafeTo(new Vector2d(-25, 6))
                .build();
        Trajectory trajRight2 = drive.trajectoryBuilder(trajRight1.end())
                .strafeTo(new Vector2d(-5, -40))
                .build();

        Trajectory trajLeft0 = drive.trajectoryBuilder(new Pose2d(-29.5, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(-29.5, 3.5)) //placeholder
                .build();
        Trajectory trajLeft1 = drive.trajectoryBuilder(trajLeft0.end())
                .strafeTo(new Vector2d(-25, 0))
                .build();
        Trajectory trajLeft2 = drive.trajectoryBuilder(trajLeft1.end())
                .strafeTo(new Vector2d(-5, -40))
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
        } else if (leftSense <2.9) { //if team object is on the LEFT
            telemetry.addData("Left", leftSense);
            telemetry.update();
            //drive.followTrajectory(trajBack);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(trajLeft0);
            drive.grabberServo(1);
            drive.setXrailPower(-.5,0);
            sleep(1000);
            drive.setXrailPower(0,0);
            drive.followTrajectory(trajLeft1);
            drive.followTrajectory(trajLeft2);

        } else if (leftSense>=2.9 && backSense >= 2.9) { //if team object is on the RIGHT
            telemetry.addData("Right", leftSense);
            telemetry.update();
            //drive.followTrajectory(trajBack);
            drive.turn(-Math.toRadians(90));
            drive.followTrajectory(trajRight0);
            drive.grabberServo(1);
            drive.setXrailPower(-.5,0);
            sleep(1000);
            drive.setXrailPower(0,0);
            drive.followTrajectory(trajRight1);
            drive.followTrajectory(trajRight2);

        } else { //if like the sun explodes idk
            telemetry.addData("?????", leftSense);
        }
        telemetry.update();
        sleep(10000);


    }

}
