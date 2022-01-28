package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedLeft")

public class RedLeft extends LinearOpMode {

    private DcMotor ferriswheel;
    private DcMotor Front_right;
    private DcMotor Back_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor linearslide;
    private Servo Right_Servo;
    private Servo Left_Servo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // TODO: Enter the type for variable named i
        double i;

        ferriswheel = hardwareMap.get(DcMotor.class, "ferriswheel");
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Right_Servo = hardwareMap.get(Servo.class, "Right_Servo");
        Left_Servo = hardwareMap.get(Servo.class, "Left_Servo");

        Inti();
        waitForStart();
        if (opModeIsActive()) {
            linearslide.setPower(0.3);
            Strafe_at_angle(0.5, 0);
            sleep(240);
            Zero_power();
            Strafe_at_angle(0.38, 270);
            sleep(700);
            Zero_power();
            Strafe_at_angle(0.5, 180);
            sleep(55);
            Zero_power();
            ferriswheel.setPower(-0.3);
            sleep(5500);
            Zero_power();
            linearslide.setPower(.5);
            Strafe_at_angle(0.5, 0);
            sleep(200);
            Zero_power();
            Strafe_at_angle(0.5, 90);
            sleep(3450);
            Zero_power();
            Strafe_at_angle(0.5, 0);
            sleep(570);
            Zero_power();
            linearslide.setPower(-.5);
            sleep(330);
            Zero_power();
            linearslide.setPower(0.1);
            Right_Servo.setPosition(1);
            Left_Servo.setPosition(1);
            sleep(300);
            Zero_power();
            linearslide.setPower(-0.4);
            Strafe_at_angle(0.5, 180);
            sleep(1020);
            Zero_power();
            linearslide.setPower(0.3);
            sleep(300);
            Zero_power();
            linearslide.setPower(0.24);
            Strafe_at_angle(0.5, 90);
            sleep(4500);
            Zero_power();
        }
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

    /**
     * Describe this function...
     */
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
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Servo.setDirection(Servo.Direction.FORWARD);
        Left_Servo.setDirection(Servo.Direction.REVERSE);
        Right_Servo.setPosition(.5);
        Left_Servo.setPosition(.5);
    }
}
