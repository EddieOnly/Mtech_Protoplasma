//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Odometry {
//
//
//    public static final Double TRIGGER_THRESHOLD = 0.5;
//
//
//    private DcMotor Front_right;
//    private DcMotor Front_left;
//    private DcMotor Back_left;
//    private DcMotor Back_right;
//
//
//    private DcMotor encoderLeft;
//    private DcMotor encoderCenter;
//    private DcMotor encoderRight;
//
//    private HardwareMap hardwareMap;
//
//    public Odometry(HardwareMap aHardwareMap) {
//
//        hardwareMap = aHardwareMap;
//        Front_right = hardwareMap.get(DcMotor.class, "FR");
//        Front_left = hardwareMap.get(DcMotor.class, "FL");
//        Back_left = hardwareMap.get(DcMotor.class, "BL");
//        Back_right = hardwareMap.get(DcMotor.class, "BR");
//
//        // Put initialization blocks here.
//        Front_right.setDirection(DcMotorSimple.Direction.FORWARD);
//        Front_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        Back_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        Back_right.setDirection(DcMotorSimple.Direction.FORWARD);
//
//
//        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        Front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        encoderLeft = Back_left;
//        encoderCenter = Front_right;
//        encoderRight = Back_right;
//
//
//        Zero_power();
//        resetDriveEncoders();
//    }
//
//    public void resetDriveEncoders() {
//
//        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public void Zero_power() {
//        Front_right.setPower(0);
//        Back_right.setPower(0);
//        Front_left.setPower(0);
//        Back_left.setPower(0);
//    }
//
//    final static double L = 0;
//    final static double B = 0;
//    final static double R = 0;
//    final static double N = 0;
//    final static double cm_per_tick = 2.0 * Math.PI * R / N;
//
//
//    public int currentRightPosition = 0;
//    public int currentLeftPosition = 0;
//    public int currentCenterPosition = 0;
//
//    private int oldRightPosition = 0;
//    private int oldLeftPosition = 0;
//    private int oldCenterPosition = 0;
//
//    public XyhVector  START_POS = new X
//
//}