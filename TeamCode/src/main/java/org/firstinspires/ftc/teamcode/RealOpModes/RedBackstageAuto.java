package org.firstinspires.ftc.teamcode.RealOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RedBackstageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double xRailRotMin = drive.xRailRot.getCurrentPosition();

        waitForStart();
        if (isStopRequested()) return;

        //drive.setPoseEstimate();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-29.5, 0))
                .build();
        Trajectory trajBack = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-50, -30))
                .build();
        /*Trajectory trajLeft = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, 0)) //placeholder
                .build();
        Trajectory trajRight = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-50, -30), 0) //placeholder
                .build();*/

        drive.followTrajectory(traj1);
        double backSense = drive.Sense(drive.colorBack);
        double leftSense = drive.Sense(drive.colorLeft);
        if (backSense < 2.9) { //if team object is at the BACK
            telemetry.addData("Back", backSense);
            drive.followTrajectory(
                    drive.trajectoryBuilder(traj1.end(), true)
                            .lineTo(new Vector2d(-25, 0))
                            .build()
            );
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(traj1);
            drive.grabberServo(1);
            drive.followTrajectory(trajBack);
        } else if (leftSense <2.9) { //if team object is on the LEFT
            telemetry.addData("Left", leftSense);
        } else if (leftSense>=2.9 && backSense >= 2.9) { //if team object is on the RIGHT
            telemetry.addData("Right", leftSense);
        } else { //if like the sun explodes idk
            telemetry.addData("?????", leftSense);
        }
        telemetry.update();
        sleep(10000);


    }

}
