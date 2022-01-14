package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp (Blocks to Java)")
@Disabled
public class ftcteleop extends LinearOpMode {

    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor Back_right;
    private DcMotor linearslide;
    private Servo Right_Servo;
    private Servo Left_Servo;
    private DcMotor ferriswheel;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double slow = 0;
        double slideidle;

        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Right_Servo = hardwareMap.get(Servo.class, "Right_Servo");
        Left_Servo = hardwareMap.get(Servo.class, "Left_Servo");
        ferriswheel = hardwareMap.get(DcMotor.class, "ferriswheel");

        // Put initialization blocks here.
        Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Servo.setDirection(Servo.Direction.FORWARD);
        Left_Servo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
        }
        while (opModeIsActive()) {
            // Put loop blocks here.
            Back_left.setPower(slow * 1 * 1 * (gamepad1.left_stick_x + (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
            Back_right.setPower(slow * (-gamepad1.left_stick_x + -gamepad1.right_stick_y + gamepad1.right_stick_x));
            Front_right.setPower(slow * (-gamepad1.left_stick_x + (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
            Front_left.setPower(slow * (gamepad1.left_stick_x + -gamepad1.right_stick_y + gamepad1.right_stick_x));
            ferriswheel.setPower(-gamepad1.right_trigger);
            ferriswheel.setPower(gamepad1.left_trigger);
            if (gamepad1.x) {
                slow = 0.35;
            }
            if (gamepad1.y) {
                slow = 1;
            }
            if (gamepad2.dpad_up) {
                linearslide.setPower(-0.9);
            } else if (gamepad2.dpad_down) {
                linearslide.setPower(0);
            } else {
                linearslide.setPower(-0.4);
            }
            if (gamepad2.x) {
                slideidle = -0.1;
            }
            if (gamepad2.y) {
                slideidle = 0;
            }
            if (gamepad2.left_bumper) {
                Right_Servo.setPosition(0.5);
                Left_Servo.setPosition(0.5);
            }
            if (gamepad2.right_bumper) {
                Right_Servo.setPosition(1);
                Left_Servo.setPosition(1);
            }
            telemetry.update();
            telemetry.addData("RUN", getRuntime());
            telemetry.addData("Back Left", Back_left.getPower());
            telemetry.addData("Back Right", Back_right.getPower());
            telemetry.addData("Front Right", Front_right.getPower());
            telemetry.addData("Front Left", Front_left.getPower());
            telemetry.addData("Ferriswheel", ferriswheel.getPower());
            telemetry.addData("Right Servo", Right_Servo.getPosition());
            telemetry.addData("Left Servo", Left_Servo.getPosition());
        }
    }
}