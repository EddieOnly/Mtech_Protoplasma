package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "andrewAuto")
public class andrewAuto extends LinearOpMode {

    private DcMotor Front_right;
    private DcMotor Back_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor linearslide;
    private CRServo Right_Servo;
    private CRServo Left_Servo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // TODO: Enter the type for variable named i

        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Right_Servo = hardwareMap.get(CRServo.class, "Right_Servo");
        Left_Servo = hardwareMap.get(CRServo.class, "Left_Servo");

        Inti();
        waitForStart();
        if (opModeIsActive()) {
            for (int i = 0; i < 10; i++) {
                Strafe_at_angle(0.5, 45);
                sleep(1000);
                Strafe_at_angle(.5, 315);
                sleep(1000);
                Strafe_at_angle(.5, 225);
                sleep(1000);
                Strafe_at_angle(.5, 135);
                sleep(1000);
                Zero_power();
                linearslide.setPower(-0.5);
                sleep(500);
                linearslide.setPower(0);
                sleep(500);
                linearslide.setPower(-0.5);
                sleep(500);
                linearslide.setPower(0);
                sleep(500);
                linearslide.setPower(-0.5);
                sleep(500);
                linearslide.setPower(0);
                sleep(500);
                Zero_power();
            }
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
    public void Inti() {
        Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Servo.setDirection(CRServo.Direction.FORWARD);
        Left_Servo.setDirection(CRServo.Direction.REVERSE);
        Right_Servo.setPower(.5);
        Left_Servo.setPower(.5);
        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}