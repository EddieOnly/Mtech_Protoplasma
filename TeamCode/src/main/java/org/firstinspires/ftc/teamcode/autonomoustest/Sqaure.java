package org.firstinspires.ftc.teamcode.autonomoustest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrains.BasicDrivetrain;

@Autonomous(name = "Automode")
public class Sqaure extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialization
        BasicDrivetrain DriveTrain = new BasicDrivetrain(hardwareMap, "Front_right", "Front_left", "Back_right", "Back_left");

        waitForStart();

        //main
        while (opModeIsActive());{
            DriveTrain.goForward(1);
            sleep(100);
            DriveTrain.goRight(1);
            sleep(100);
            DriveTrain.goBackward(1);
            sleep(100);
            DriveTrain.goRight(1);
            sleep(100);
        }
    }
}
