package org.firstinspires.ftc.teamcode.RealOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@TeleOp(name = "Teleop1")
public class Teleop1 extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            //mecanum drive w/ precision mode
            if (gamepad1.left_bumper) {
                drive.mecanumDrive(0.5 * gamepad1.right_stick_y, -0.5 * gamepad1.right_stick_x, -0.5 * gamepad1.left_stick_x);
            } else if (gamepad1.right_bumper) {
                drive.mecanumDrive(0.25 * gamepad1.right_stick_y, -0.25 * gamepad1.right_stick_x, -0.25 * gamepad1.left_stick_x);
            } else {
                drive.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            }
            //start
            //This code below may be useful for coding our intake and drops
        /*
        // lift servo (BOX)
        if (gamepad2.y) {
            robot.liftServo(1); //dumps stuff out
        } else if (gamepad2.x) {
            robot.liftServo(0); //
        } else if (gamepad2.a) {
            robot.liftServo(0.2); //straight up and down
        } else if (gamepad2.b) {
            robot.liftServo(0.4); //tilt
        }
        */


        }
    }



}
