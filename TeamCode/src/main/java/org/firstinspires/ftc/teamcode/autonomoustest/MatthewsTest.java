package org.firstinspires.ftc.teamcode.autonomoustest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MatthewsPetTrash")
public class MatthewsTest extends LinearOpMode {
    DcMotor Frontleft;
    DcMotor Frontright;
    DcMotor Backleft;
    DcMotor Backright;
    DcMotor farriswheel;
    public void runOpMode() throws InterruptedException {
        //Init
        Frontleft = hardwareMap.dcMotor.get("Front_left");
        Frontright = hardwareMap.dcMotor.get("Front_right");
        Backleft = hardwareMap.dcMotor.get("Back_left");
        Backright = hardwareMap.dcMotor.get("Back_right");
        farriswheel = hardwareMap.dcMotor.get("farriswheel");
        waitForStart();
        //main
        if (opModeIsActive()) {
            Frontright.setPower(1);
            Frontleft.setPower(1);
            Backright.setPower(1);
            Backleft.setPower(1);
            Thread.sleep(500);


        }

    }
}
