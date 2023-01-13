package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Test")
public class linearslidetest extends LinearOpMode {
    public DcMotor Lift;

    public void runOpMode() throws InterruptedException {

        Lift = hardwareMap.get(DcMotor.class, "LL");

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        final int SLIDE_INITIAL = Lift.getCurrentPosition();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            slideTo(SLIDE_INITIAL + 3, 0.5);
            telemetry.addData("sleeping 1", "");
            telemetry.update();
            sleep(1000);
            slideTo(SLIDE_INITIAL, 0.5);
            telemetry.addData("sleeping 2", "");
            telemetry.update();
            sleep(1000);
            slideTo(SLIDE_INITIAL + 2, 0.5);
            telemetry.addData("sleeping 3", "");
            telemetry.update();
            sleep(1000);
            slideTo(SLIDE_INITIAL, 0.5);
            telemetry.addData("sleeping 4", "");
            telemetry.update();
            sleep(1000);

        }
    }

    public void Lift_Up(double power, int distance) {

        // reset encoders
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Lift.setTargetPosition(distance);

        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Lift.setPower(power);

        while (opModeIsActive() && Lift.isBusy()) {
            telemetry.addData("Lift", "Running to %7d", distance);
            telemetry.update();

            idle();
        }
    }


    public void Lift_Down(double power, int distance) {

        // reset encoders
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Lift.setTargetPosition(-distance);

        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Lift.setPower(power);

        while (opModeIsActive() && Lift.isBusy()) {
            telemetry.addData("Lift", "Running to %7d", -distance);
            telemetry.update();

            idle();
        }

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int slideTo(int targetPosition, double power) {
        Lift.setTargetPosition(targetPosition);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(power);
        while (opModeIsActive() && Lift.isBusy()) {
            telemetry.addData("Lift", "Running to %7d", targetPosition);
            telemetry.update();
        }
        return targetPosition;
    }
}

