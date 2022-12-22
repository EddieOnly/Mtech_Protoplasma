package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Ariella on 6/10/2017.
 */
@Disabled
@Autonomous(name = "Encoder Test", group = "Pushbot")

public class Encodertest extends AutoEncoder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        TurnLeft(.4, 5);


    }
}