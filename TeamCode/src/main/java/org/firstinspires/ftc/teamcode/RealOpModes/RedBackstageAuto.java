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
                .forward(38.5)
                .build();
        Trajectory traj2pos1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0,10),Math.toRadians(180))
                .build();

        drive.followTrajectory(traj1);
        double frontSense = drive.Sense(drive.colorFront);
        double leftSense = drive.Sense(drive.colorLeft);
        if (frontSense < 4.0) {
            telemetry.addData("A", frontSense);
        } else if (leftSense <4.0) {
            telemetry.addData("B", leftSense);
        } else if (leftSense>=4.0 && frontSense >= 4.0) {
            telemetry.addData("C", leftSense);
        } else {
            telemetry.addData("D", leftSense);
        }
        telemetry.update();
        sleep(300);


    }

}
