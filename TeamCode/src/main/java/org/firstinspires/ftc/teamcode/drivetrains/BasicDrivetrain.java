package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicDrivetrain {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    public BasicDrivetrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft) {
        this.frontLeft = hardwareMap.dcMotor.get(frontLeft);
        this.backRight = hardwareMap.dcMotor.get(backRight);
        this.backLeft = hardwareMap.dcMotor.get(backLeft);
        this.frontRight = hardwareMap.dcMotor.get(frontRight);

    }

    public void moveAtAngle(double angle, double power) {

        double RightSidePower, LeftSidePower;

        angle = angle % 360;

        RightSidePower = -Math.cos(Math.toRadians(angle + 45.0));
        LeftSidePower = -Math.cos(Math.toRadians(angle - 45));

        RightSidePower = power * RightSidePower;
        LeftSidePower = power * LeftSidePower;

        frontRight.setPower(RightSidePower);
        backLeft.setPower(RightSidePower);

        frontLeft.setPower(LeftSidePower);
        backRight.setPower(LeftSidePower);

    }

    public void turnAtPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);

    }

    public void goForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void goBackward(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void goRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void goLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }


}
