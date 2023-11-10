package org.firstinspires.ftc.teamcode.RealOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(10, 0))
                .build();

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj1);

    }
}
