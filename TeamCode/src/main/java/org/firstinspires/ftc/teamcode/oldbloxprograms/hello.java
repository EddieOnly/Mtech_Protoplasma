package org.firstinspires.ftc.teamcode.oldbloxprograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "hello (Blocks to Java)")
public class hello extends LinearOpMode {

  private DcMotor Front_right;
  private DcMotor Front_left;
  private DcMotor Back_left;
  private DcMotor Back_right;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    Front_right = hardwareMap.get(DcMotor.class, "Front_right");
    Front_left = hardwareMap.get(DcMotor.class, "Front_left");
    Back_left = hardwareMap.get(DcMotor.class, "Back_left");
    Back_right = hardwareMap.get(DcMotor.class, "Back_right");

    // Put initialization blocks here.
    Front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
    Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    Back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put loop blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        Back_left.setPower(-gamepad1.left_stick_y);
        Back_right.setPower(-gamepad1.right_stick_y);
        Front_right.setPower(-gamepad1.right_stick_y);
        Front_left.setPower(-gamepad1.left_stick_y);
        Front_left.setPower(1);
        telemetry.addData("Front_Left Power", 123);
        telemetry.addData("RUN", getRuntime());
        telemetry.update();
      }
    }
  }
}
