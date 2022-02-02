package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotsubsystems.Spinner;

@Autonomous(name = "Encoder")

public class EncoderDriveBaseTest extends LinearOpMode {
    private DcMotor ferriswheel;
    private DcMotor Front_right;
    private DcMotor Back_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor linearslide;
    private Servo Right_Servo;
    private Servo Left_Servo;

    @Override
    public void runOpMode() throws InterruptedException {
        ferriswheel = hardwareMap.get(DcMotor.class, "ferriswheel");
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Right_Servo = hardwareMap.get(Servo.class, "Right_Servo");
        Left_Servo = hardwareMap.get(Servo.class, "Left_Servo");
    }



}

