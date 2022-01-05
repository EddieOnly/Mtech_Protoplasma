package org.firstinspires.ftc.teamcode.autonomoustest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrains.BasicDrivetrain;

@Autonomous(name = "Automode")
public class TestAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialization
        BasicDrivetrain DriveTrain = new BasicDrivetrain(hardwareMap, "Front_right", "Front_left", "Back_right", "Back_left");

        waitForStart();

        //main
        DriveTrain.moveAtAngle(45, 1);
        Thread.sleep(2000);
        DriveTrain.stop();


    }
}
