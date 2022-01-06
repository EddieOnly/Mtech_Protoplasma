package org.firstinspires.ftc.teamcode.oldbloxprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Movement")
public class Movement extends LinearOpMode {
    private DcMotor Front_left;
    private DcMotor Front_right;
    private DcMotor Back_left;
    private DcMotor Back_right;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.left_stick_x <= 0 && gamepad1.right_stick_x <= 0){
                Front_right.setPower(-gamepad1.left_stick_x);
                Front_left.setPower(-gamepad1.left_stick_x);
                Back_left.setPower(gamepad1.left_stick_x);
                Back_right.setPower(gamepad1.right_stick_x);
            }
            else if (gamepad1.left_stick_x >= 0 && gamepad1.right_stick_x >= 0){
                Front_right.setPower(-gamepad1.left_stick_x);
                Front_left.setPower(-gamepad1.left_stick_x);
                Back_left.setPower(gamepad1.left_stick_x);
                Back_right.setPower(gamepad1.right_stick_x);
            if (gamepad1.left_stick_y <= 0) {
                Front_left.setPower(gamepad1.left_stick_y);
                Back_left.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.left_stick_y > 0){
                Front_left.setPower(gamepad1.left_stick_y);
                Back_left.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.right_stick_y <= 0){
                Front_right.setPower(-gamepad1.right_stick_y);
                Back_right.setPower(-gamepad1.right_stick_y);
            } else if (gamepad1.right_stick_y > 0){
                Front_right.setPower(-gamepad1.right_stick_y);
                Back_right.setPower(-gamepad1.right_stick_y);
            }
            }
        }
    }
}
