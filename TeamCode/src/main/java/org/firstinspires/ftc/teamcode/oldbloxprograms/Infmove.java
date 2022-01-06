package org.firstinspires.ftc.teamcode.oldbloxprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "MatthewSinghAutonomousHahaMickandeddiearedoingnothingatallrightnow (Blocks to Java)")
@Disabled
public class MatthewSinghAutonomousHahaMickandeddiearedoingnothingatallrightnow extends LinearOpMode {

    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor Back_right;
    private DcMotor linearslide;
    private Servo Right_Servo;
    private Servo Left_Servo;

    double Distance;
    double i;


    public void runOpMode() {
        double Angle;

        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Right_Servo = hardwareMap.get(Servo.class, "Right_Servo");
        Left_Servo = hardwareMap.get(Servo.class, "Left_Servo");

        if (opModeIsActive()) {
            Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
            Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
            linearslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Servo.setDirection(Servo.Direction.FORWARD);
            Left_Servo.setDirection(Servo.Direction.REVERSE);
            waitForStart();
            while (opModeIsActive()) {
                double i_end = Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885;
                double i_inc = 1;
                if (1 > i_end) {
                    i_inc = -i_inc;
                }
                for (i = 1; i_inc >= 0 ? i <= i_end : i >= i_end; i += i_inc) {
                    move(null, null);
                    Strafe_at_angle(null, null);
                    Front_right.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
                }
                double i_end2 = Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885;
                double i_inc2 = 1;
                if (1 > i_end2) {
                    i_inc2 = -i_inc2;
                }
                for (i = 1; i_inc2 >= 0 ? i <= i_end2 : i >= i_end2; i += i_inc2) {
                    move(null, null);
                    Strafe_at_angle(null, null);
                    Front_left.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
                }
                double i_end3 = Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885;
                double i_inc3 = 1;
                if (1 > i_end3) {
                    i_inc3 = -i_inc3;
                }
                for (i = 1; i_inc3 >= 0 ? i <= i_end3 : i >= i_end3; i += i_inc3) {
                    move(null, null);
                    Strafe_at_angle(null, null);
                    Back_right.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
                }
                double i_end4 = Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885;
                double i_inc4 = 1;
                if (1 > i_end4) {
                    i_inc4 = -i_inc4;
                }
                for (i = 1; i_inc4 >= 0 ? i <= i_end4 : i >= i_end4; i += i_inc4) {
                    move(null, null);
                    Strafe_at_angle(null, null);
                    Back_left.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
                }
                Reset_encoders();
            }
        }
        // Put initialization blocks here.
    }

    /**
     * Describe this function...
     */
    private void move(double Distance, double Angle) {
        double X2;
        double Y2;

        Reset_encoders();
        X2 = Distance * Math.sin(Angle / 180 * Math.PI);
        Y2 = Distance * Math.cos(Angle / 180 * Math.PI);
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
        double power_1;
        double power_2;

        Angle = Math.min(Math.max(Angle, 0), 360);
        power_1 = -Math.cos((Angle + 45) / 180 * Math.PI);
        power_2 = -Math.cos((Angle - 45) / 180 * Math.PI);
        power_1 = Power * power_1;
        power_2 = Power * power_2;
        Front_right.setPower(power_1);
        Back_left.setPower(power_1);
        Front_left.setPower(power_2);
        Back_right.setPower(power_2);
        Front_right.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
    }
}
