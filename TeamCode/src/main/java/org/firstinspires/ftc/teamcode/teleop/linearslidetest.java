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
        Lift = hardwareMap.get(DcMotor.class, "LL");

        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setPower(.4);

        waitForStart();

        while (opModeIsActive()) {
            //Lift_High(.4, 250);
            Lift.setTargetPosition(250);
            while (opModeIsActive() && Lift.isBusy()) {
                idle();
            }

            Thread.sleep(5000);

            Lift.setTargetPosition(0);
            while (opModeIsActive() && Lift.isBusy()) {
                idle();
            }
        }
    }

    public void Lift_High(double power, int distance) {

        // reset encoders
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Lift.setTargetPosition(distance);

        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Lift.setPower(power);

        while (opModeIsActive() && Lift.isBusy()) {
            telemetry.addData("Lift", "Running to %7d", distance);
            telemetry.update();

            idle();
        }

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

