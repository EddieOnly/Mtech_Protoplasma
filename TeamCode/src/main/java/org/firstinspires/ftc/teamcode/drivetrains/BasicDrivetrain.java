package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class BasicDrivetrain extends LinearOpMode{
    private DcMotor Front_right;
    private DcMotor Back_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor linearslide;
    private Servo Left_Servo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    public void runOpMode() {
        // TODO: Enter the type for variable named i
        double i;

        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Left_Servo = hardwareMap.get(Servo.class, "Left_Servo");

    }

    /**
     * Describe this function...
     */
    private void Zero_power() {
        Front_right.setPower(0);
        Back_right.setPower(0);
        Front_left.setPower(0);
        Back_left.setPower(0);
    }

    private void move(double Distance, double Angle) {
        double Y2;
        double X2;

        Reset_encoders();
        Y2 = Distance * Math.sin(Angle / 180 * Math.PI);
        X2 = Distance * Math.cos(Angle / 180 * Math.PI);
        Strafe_at_angle(0.8, Angle);
    }

    /**
     * Describe this function...
     */
    private void Reset_encoders() {
        Back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void Strafe_at_angle(double Power, double Angle) {
        double Distance = 0;
        double power_1;
        double power_2;

        Angle = Math.min(Math.max(Angle, 0), 360);
        power_1 = Math.cos((Angle + 45) / 180 * Math.PI);
        power_2 = Math.cos((Angle - 45) / 180 * Math.PI);
        power_1 = Power * power_1;
        power_2 = Power * power_2;
        Front_right.setPower(power_1);
        Back_left.setPower(power_1);
        Front_left.setPower(power_2);
        Back_right.setPower(power_2);
        Front_right.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
        Front_left.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
        Back_right.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
        Back_left.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
    }

    /**
     * Describe this function...
     */
    private void Inti() {
        Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        linearslide.setDirection(DcMotorSimple.Direction.REVERSE);
        Left_Servo.setDirection(Servo.Direction.REVERSE);
        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
