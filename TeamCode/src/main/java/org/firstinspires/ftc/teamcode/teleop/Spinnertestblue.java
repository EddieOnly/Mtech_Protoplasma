package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotsubsystems.Spinner;

@Autonomous(name="bluesidespinner")
public class Spinnertestblue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialization
        Spinner Ferriswheel = new Spinner(hardwareMap, "ferriswheel");
        waitForStart();
        //main

        if (opModeIsActive()) {
            Ferriswheel.blue(.45);


        }
    }
}