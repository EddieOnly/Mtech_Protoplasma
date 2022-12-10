package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.teamcode.drivetrains.BasicDrivetrain;
@Disabled
@TeleOp(name = "andrewTeleOp")

public class andrewTeleOP extends BasicDrivetrain {
    double speed = 1;
    int ws = 3;
    public DcMotor Front_right;
    public DcMotor Back_right;
    public DcMotor Front_left;
    public DcMotor Back_left;
    public DcMotor linearslide;
    public CRServo Left_Servo;
    public CRServo Right_Servo;
    boolean has_target_set = false;


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Left_Servo = hardwareMap.get(CRServo.class, "Left_Servo");
        Right_Servo = hardwareMap.get(CRServo.class, "Right_Servo");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");

//        linearslide.setTargetPosition(0);
//        linearslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Servo.setDirection(CRServo.Direction.REVERSE);
        Right_Servo.setDirection(CRServo.Direction.FORWARD);
        linearslide.setDirection(DcMotorSimple.Direction.FORWARD);

        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();



        if (opModeIsActive())

            while (opModeIsActive()) {

                Back_left.setPower(speed * 1 * 1 * (gamepad1.right_stick_x + (-gamepad1.left_stick_y - gamepad1.left_stick_x)));
                Back_right.setPower(speed * (-gamepad1.right_stick_x + -gamepad1.left_stick_y + gamepad1.left_stick_x));
                Front_right.setPower(speed * (-gamepad1.right_stick_x + (-gamepad1.left_stick_y - gamepad1.left_stick_x)));
                Front_left.setPower(speed * (gamepad1.right_stick_x + -gamepad1.left_stick_y + gamepad1.left_stick_x));

//                if (gamepad1.right_bumper) {
//                    speed = 1;
//                } else if (gamepad1.left_bumper) {
//                    speed = 0.2;
//                } else {
//                    speed = 0.5;
//                }

                if (gamepad1.right_bumper) {
                    speed = 0.3;
                } else {
                    speed = 0.5 + gamepad1.right_trigger/2;
                }

                telemetry.update();
                telemetry.addData("RUN", getRuntime());
                telemetry.addData("Back Left", Back_left.getPower());
                telemetry.addData("Back Right", Back_right.getPower());
                telemetry.addData("Front Right", Front_right.getPower());
                telemetry.addData("Front Left", Front_left.getPower());
                telemetry.addData("linear slide", linearslide.getPower());
                telemetry.addData("Left Servo", Left_Servo.getPower());
                telemetry.addData("Right_Servo", Right_Servo.getPower());
            }
    }
}