package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="AutoRedBackstage")
@Disabled
public class AutoRedBackstage extends LinearOpMode {


    //Declare OpMode members

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private ElapsedTime runtime = new ElapsedTime();
    //can use visionPortal.stopStreaming(); or visionPortal.resumeStreaming(); after the opencv runs




    @Override
    public void runOpMode() throws InterruptedException {



    }


}
