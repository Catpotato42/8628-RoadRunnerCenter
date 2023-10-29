package org.firstinspires.ftc.teamcode.RealOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@TeleOp(name = "Teleop1")
public class Teleop1 extends OpMode {
        double xRailPower;
        double Sensed;
        public void init() {
            xRailPower = .5;
        }


        public void loop() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //mecanum drive w/ precision mode
            if (gamepad1.left_bumper) {
                drive.mecanumDrive(0.5 * gamepad1.right_stick_y, -0.5 * gamepad1.right_stick_x, -0.5 * gamepad1.left_stick_x);
            } else if (gamepad1.right_bumper) {
                drive.mecanumDrive(0.25 * gamepad1.right_stick_y, -0.25 * gamepad1.right_stick_x, -0.25 * gamepad1.left_stick_x);
            } else {
                drive.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            }

            if(gamepad2.left_bumper) {
                drive.setXrailPower(gamepad2.right_stick_y*.5, gamepad2.left_stick_y*.5);
            } else {
                drive.setXrailPower(gamepad2.right_stick_y, gamepad2.left_stick_y);
            }
            //start
            //This code below may be useful for coding our intake and drops

            // open servo
            if (gamepad2.y) {
            drive.grabberServo(1); //grabs
            } else if (gamepad2.x) {
            drive.grabberServo(0); //dumps stuff out
            } else if (gamepad2.a) {
                Sensed = drive.Sense(drive.colorFront); //test1
                telemetry.addData("Distance: ", Sensed);
            } else if (gamepad2.b) {
            drive.grabberServo(0.4); //test2
            }
            telemetry.update();

        }




}
