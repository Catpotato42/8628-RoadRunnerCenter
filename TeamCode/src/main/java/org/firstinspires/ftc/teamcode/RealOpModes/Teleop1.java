package org.firstinspires.ftc.teamcode.RealOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@TeleOp(name = "Teleop1")
public class Teleop1 extends OpMode {
        double xRailPower;
        double Sensed;
        double sensedColor;
        private ElapsedTime elapsedRunTime = new ElapsedTime();
        double xRailRotMin;
        boolean StartTime = true;
        int Infractions = 0;
        public void init() {

            xRailPower = 1;

        }


        public void loop() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            if (StartTime) {
                drive.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //REDUNDANCY!!!
                telemetry.addData("I ran ", drive.xRailRot.getCurrentPosition());
                telemetry.update();
                xRailRotMin = drive.xRailRot.getCurrentPosition();
                StartTime = false;
            }
            //mecanum drive w/ precision mode
            if (gamepad1.left_bumper) {
                drive.mecanumDrive(-.7 * gamepad1.right_stick_y, .7 * gamepad1.right_stick_x, .7 * gamepad1.left_stick_x);
            } else if (gamepad1.right_bumper) {
                drive.mecanumDrive(-0.15 * gamepad1.right_stick_y, 0.15 * gamepad1.right_stick_x, 0.15 * gamepad1.left_stick_x);
            } else {
                drive.mecanumDrive(-0.25 * gamepad1.right_stick_y, 0.25 * gamepad1.right_stick_x, 0.25 * gamepad1.left_stick_x);
            }

            if (gamepad2.right_bumper && drive.xRailRot.getCurrentPosition() < (xRailRotMin+10)) {
                drive.setXrailPower(0.75 * gamepad2.right_stick_y, 0.75 * -gamepad2.left_stick_y);
                telemetry.addData("takin it slow ;", 0);
            } else if (drive.xRailRot.getCurrentPosition() < (xRailRotMin+10)) {
                drive.setXrailPower(gamepad2.right_stick_y, -gamepad2.left_stick_y);
                telemetry.addData("normal :", 0);
            } else if (gamepad2.left_bumper) {
                drive.setXrailPower(gamepad2.right_stick_y, -gamepad2.left_stick_y);
                telemetry.addData("override :", 8);
            } else {
                drive.setXrailPower(-0.1, -gamepad2.left_stick_y);
                telemetry.addData("goin up :( ZER", 0);
            }
            //start
            //This code below may be useful for coding our intake and drops
            if (gamepad1.dpad_up) {
                drive.droneServo.setPosition(1);
            } else if (gamepad2.dpad_down) {
                drive.hangerServo.setPosition(1);
            } else if (gamepad1.dpad_left) {
                drive.droneServo.setPosition(0);
            } else if (gamepad2.dpad_right) {
                drive.hangerServo.setPosition(0);
            }

            // open servo
            if (gamepad2.y) {
            drive.grabberServo(1); //release
            } else if (gamepad2.x) {
            drive.grabberServo(0); //grab
            } else if (gamepad2.a) {
                Sensed = drive.Sense(drive.colorBack); //test1
                sensedColor = drive.colorBack.green();
                telemetry.addData("Distance: ", Sensed);
                telemetry.addData("Distance: ", sensedColor);
            } else if (gamepad2.b) {
                drive.xRailRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.xRailRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                xRailRotMin = drive.xRailRot.getCurrentPosition();
                Infractions++;
                telemetry.addData("Reset encoders: ", Infractions);
            }
            telemetry.addData("Extender",drive.xRailExt.getCurrentPosition());
            telemetry.addData("Rotater",drive.xRailRot.getCurrentPosition());
            telemetry.update();

        }




}
