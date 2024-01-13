package org.firstinspires.ftc.teamcode.RealOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Teleop1")
public class Teleop1 extends OpMode {
        double xRailPower;
        double Sensed;
        double sensedColor;
        private ElapsedTime elapsedRunTimeBack = new ElapsedTime();
        private ElapsedTime elapsedRunTimeFront = new ElapsedTime();
        double xRailRotMin;
        double xRailExtMax;
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
                xRailRotMin = 0;
                xRailExtMax = 9600;
                StartTime = false;
            }
            //mecanum drive w/ precision mode
            if (gamepad1.left_bumper) {
                drive.mecanumDrive(-1 * gamepad1.right_stick_y, 1 * gamepad1.right_stick_x, 1 * gamepad1.left_stick_x);
            } else if (gamepad1.right_bumper) {
                drive.mecanumDrive(-0.25 * gamepad1.right_stick_y, 0.25 * gamepad1.right_stick_x, 0.25 * gamepad1.left_stick_x);
            } else {
                drive.mecanumDrive(-.5 * gamepad1.right_stick_y, 0.5 * gamepad1.right_stick_x, 0.5 * gamepad1.left_stick_x);
            }
            telemetry.addData("current right trigger: ", gamepad1.right_trigger);
            telemetry.addData("current left trigger: ", gamepad1.left_trigger);
            telemetry.update();
            if (gamepad1.right_trigger > .01) {
                drive.setXrailPower(-gamepad1.right_trigger, -gamepad2.left_stick_y);
            } else if (gamepad1.left_trigger > .01) {
                drive.setXrailPower(gamepad1.left_trigger, -gamepad2.left_stick_y);
            } else if (drive.xRailRot.getCurrentPosition() < (xRailRotMin+3) && drive.xRailExt.getCurrentPosition() < (xRailExtMax - 3)) {
                drive.setXrailPower(gamepad2.right_stick_y, -gamepad2.left_stick_y);
                telemetry.addData("normal :", 0);
            } else if (gamepad2.left_bumper || gamepad2.right_bumper) {
                drive.setXrailPower(gamepad2.right_stick_y, -gamepad2.left_stick_y);
                telemetry.addData("override :", 8);
            } else if (drive.xRailRot.getCurrentPosition() < (xRailRotMin+3) && drive.xRailExt.getCurrentPosition() > (xRailExtMax - 3)) {
                drive.setXrailPower(gamepad2.right_stick_y, -.5);
                telemetry.addData("goin back :( ZER", 0);
            } else if (drive.xRailRot.getCurrentPosition() > (xRailRotMin+3) && drive.xRailExt.getCurrentPosition() < (xRailExtMax - 3)) {
                drive.setXrailPower(-0.1, -gamepad2.left_stick_y);
                telemetry.addData("goin up :( ZER", 0);
            } else if (drive.xRailRot.getCurrentPosition() > (xRailRotMin+3) && drive.xRailExt.getCurrentPosition() > (xRailExtMax - 3)) {
                drive.setXrailPower(-0.1, -.5);
            }
            //start
            //This code below may be useful for coding our intake and drops
            if (gamepad1.dpad_up) {
                drive.droneServo.setPosition(1); //shoot drone
            } else if (gamepad2.dpad_down) {
                drive.hangerServo.setPosition(0); //release hanger
            } else if (gamepad1.dpad_left) {
                drive.droneServo.setPosition(0); //reset drone servo
            }


            // open servo
            if (gamepad1.y && drive.grabberServoFront.getPosition() <.5 && (elapsedRunTimeFront.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeFront.reset();

                drive.grabberServoFront(1); //release
            } else if (gamepad1.y && (elapsedRunTimeFront.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeFront.reset();

                drive.grabberServoFront(0); //grab
            } else if (gamepad1.x && drive.grabberServoBack.getPosition() <.5 && (elapsedRunTimeBack.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeBack.reset();

                drive.grabberServoBack(1); //release
            } else if (gamepad1.x && (elapsedRunTimeBack.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeBack.reset();

                drive.grabberServoBack(0); //grab
            }

            // open servo
            if (gamepad2.y && drive.grabberServoFront.getPosition() <.5 && (elapsedRunTimeFront.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeFront.reset();

                drive.grabberServoFront(1); //release
            } else if (gamepad2.y && (elapsedRunTimeFront.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeFront.reset();

                drive.grabberServoFront(0); //grab
            } else if (gamepad2.x && drive.grabberServoBack.getPosition() <.5 && (elapsedRunTimeBack.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeBack.reset();

                drive.grabberServoBack(1); //release
            } else if (gamepad2.x && (elapsedRunTimeBack.time(TimeUnit.MILLISECONDS)) > 200) {
                elapsedRunTimeBack.reset();

                drive.grabberServoBack(0); //grab
            } else if (gamepad2.a) {

            } else if (gamepad2.b) {

            } else if (gamepad2.dpad_left) {
                telemetry.addData("Extender", drive.xRailExt.getCurrentPosition());
                telemetry.addData("Rotater", drive.xRailRot.getCurrentPosition());
                telemetry.update();
                drive.xRailRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.xRailRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                xRailRotMin = drive.xRailRot.getCurrentPosition();
                Infractions++;
                telemetry.addData("Reset encoders: ", Infractions);
            } else if (gamepad2.dpad_up) {
                telemetry.addData("Extender", drive.xRailExt.getCurrentPosition());
                telemetry.addData("Rotater", drive.xRailRot.getCurrentPosition());
                telemetry.update();
                drive.xRailExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.xRailExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                xRailExtMax = drive.xRailExt.getCurrentPosition();
                Infractions++;
                telemetry.addData("Reset encoders: ", Infractions);
            }
        }
}
