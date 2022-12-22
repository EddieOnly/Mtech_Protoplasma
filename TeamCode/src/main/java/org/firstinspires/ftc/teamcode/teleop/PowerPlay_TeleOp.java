package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TELEOP FOR COMPETITION")
public class PowerPlay_TeleOp extends LinearOpMode {

    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor Back_right;
    private DcMotor linearSlide_right;
    private DcMotor linearSlide_left;
    public CRServo In_Right;
    public CRServo In_Left;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        double Speed = 0.5;

        Front_right = hardwareMap.get(DcMotor.class, "FR");
        Front_left = hardwareMap.get(DcMotor.class, "FL");
        Back_left = hardwareMap.get(DcMotor.class, "BL");
        Back_right = hardwareMap.get(DcMotor.class, "BR");
        linearSlide_right = hardwareMap.get(DcMotor.class, "LR");
        linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
        In_Right = hardwareMap.get(CRServo.class, "IRS");
        In_Left = hardwareMap.get(CRServo.class, "ILS");

        // Put initialization blocks here.
        Front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_left.setDirection(DcMotorSimple.Direction.FORWARD);
        In_Right.setDirection(CRServo.Direction.REVERSE);
        In_Left.setDirection(CRServo.Direction.FORWARD);

        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide_right.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        waitForStart();

        while (opModeIsActive()) {

            Back_left.setPower(Speed * (-gamepad1.right_stick_x + (-gamepad1.left_stick_y - gamepad1.left_stick_x)));
            Back_right.setPower(Speed * (gamepad1.right_stick_x + -gamepad1.left_stick_y + gamepad1.left_stick_x));
            Front_right.setPower(Speed * 0.85 * (gamepad1.right_stick_x + (-gamepad1.left_stick_y - gamepad1.left_stick_x)));
            Front_left.setPower(Speed * 0.85 * (-gamepad1.right_stick_x + -gamepad1.left_stick_y + gamepad1.left_stick_x));


            linearSlide_right.setPower(gamepad2.left_stick_y * 0.8);
            linearSlide_left.setPower(gamepad2.left_stick_y * 0.8);
//
//            linearSlide_left.setPower(-0.2);
//            linearSlide_right.setPower(-0.2);


            if (gamepad1.y) {
                Speed = 0.85;
            } else Speed = 0.35 + gamepad1.right_trigger / 2;

            if (gamepad2.b) {
                In_Left.setPower(-0.8);
                In_Right.setPower(-0.8);
            } else if (gamepad2.x) {
                In_Right.setPower(0.8);
                In_Left.setPower(0.8);
            } else {
                In_Left.setPower(0);
                In_Right.setPower(0);
            }


            telemetry.update();
            telemetry.addData("RUN", getRuntime());
            telemetry.addData("Back Left", Back_left.getPower());
            telemetry.addData("Back Right", Back_right.getPower());
            telemetry.addData("Front Right", Front_right.getPower());
            telemetry.addData("Front Left", Front_left.getPower());
            telemetry.addData("Right Lift", linearSlide_right.getPower());
            telemetry.addData("Left Lift", linearSlide_left.getPower());
            telemetry.addData("Right Servo", linearSlide_right.getPower());
            telemetry.addData("Left Servo", linearSlide_left.getPower());
        }
    }
}

