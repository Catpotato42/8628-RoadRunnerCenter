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

        waitForStart();
        if (isStopRequested()) return;


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        Trajectory traj2pos1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(5,0),Math.toRadians(180))
                .build();

        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2pos1);

    }

}
