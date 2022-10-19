package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OnlyTeleOp")
public class freightfrenzyteleop extends LinearOpMode {

    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor Back_right;
    private Servo Left_Servo;
    private DcMotor linearslide;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        double slow = 0;

        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Left_Servo = hardwareMap.get(Servo.class, "Left_Servo");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");

        // Put initialization blocks here.
        Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Left_Servo.setDirection(Servo.Direction.REVERSE);
        linearslide.setDirection(DcMotorSimple.Direction.REVERSE);

        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (opModeIsActive())
            linearslide.setPower(-0.5);
        sleep(1000);
        Left_Servo.setPosition(-0.5);
        while (opModeIsActive()) {

            Back_left.setPower(slow * 1 * 1 * (gamepad1.right_stick_x + (-gamepad1.left_stick_y - gamepad1.left_stick_x)));
            Back_right.setPower(slow * (-gamepad1.right_stick_x + -gamepad1.left_stick_y + gamepad1.left_stick_x));
            Front_right.setPower(slow * (-gamepad1.right_stick_x + (-gamepad1.left_stick_y - gamepad1.left_stick_x)));
            Front_left.setPower(slow * (gamepad1.right_stick_x + -gamepad1.left_stick_y + gamepad1.left_stick_x));

            if (gamepad2.dpad_up) {
                linearslide.setPower(-1);
            }
            else if(gamepad2.dpad_down) {
                linearslide.setPower(0.1);
            } else {
                linearslide.setPower(-0.28);
            }
            if (gamepad1.x) {
                slow = 0.35;
            }
            if (gamepad1.y) {
                slow = 1;
            }
            if (gamepad2.left_bumper){
                Left_Servo.setPosition(0.5);
            }
            if(gamepad2.right_bumper){
                Left_Servo.setPosition(-0.5);
            }

            telemetry.update();
            telemetry.addData("RUN", getRuntime());
            telemetry.addData("Back Left", Back_left.getPower());
            telemetry.addData("Back Right", Back_right.getPower());
            telemetry.addData("Front Right", Front_right.getPower());
            telemetry.addData("Front Left", Front_left.getPower());
            telemetry.addData("Left Servo", Left_Servo.getPosition());
            telemetry.addData("linear slide", linearslide.getPower());
        }
    }
}