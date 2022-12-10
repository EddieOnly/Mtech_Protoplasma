package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "Linearslide Test")
public class linearslidetest extends LinearOpMode {
    public DcMotor Lift;

    public void runOpMode() throws InterruptedException {
        Lift = hardwareMap.get(DcMotor.class, "L");
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setPower(-0.7); //Or whatever power is forward/up
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);        waitForStart();



        while (opModeIsActive()) {

            if (gamepad1.a) {
                Lift.setTargetPosition(0);
            }
            if (gamepad1.x) {
                Lift.setTargetPosition(100);
            }

            telemetry.update();
            telemetry.addData("Lift To", Lift.getTargetPosition());
        }
    }
}
