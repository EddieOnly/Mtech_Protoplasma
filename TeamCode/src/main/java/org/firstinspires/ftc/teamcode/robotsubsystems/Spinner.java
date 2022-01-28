package org.firstinspires.ftc.teamcode.robotsubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Spinner {
    DcMotor ferriswheel;
    VoltageSensor voltageSensor1;

    public Spinner(HardwareMap hardwareMap, String ferriswheel) {
        this.ferriswheel = hardwareMap.dcMotor.get(ferriswheel);

        for (VoltageSensor v: hardwareMap.voltageSensor){
            this.voltageSensor1 = v;
        }
    }

    public void blue(double power) throws InterruptedException {
        ferriswheel.setPower(power);
        Thread.sleep(2150);
    }

    public void red(double power) throws InterruptedException {
        ferriswheel.setPower(-power);
        Thread.sleep(2150);

    }
    public void stop(double power) throws InterruptedException {
        ferriswheel.setPower(0);
    }

}
