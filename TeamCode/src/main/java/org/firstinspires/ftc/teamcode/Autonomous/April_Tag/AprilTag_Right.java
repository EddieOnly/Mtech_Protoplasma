package org.firstinspires.ftc.teamcode.Autonomous.April_Tag;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RIGHT AUTO FOR COMPETITION", group = "AUTO")
public class AprilTag_Right extends LinearOpMode {
    public DcMotor Front_right;
    public DcMotor Front_left;
    public DcMotor backRight;
    public DcMotor backLeft;


    static final double COUNTS_PER_MOTOR_REV = 537;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //calculations
    double diameterOfWheel = 3.96; //inches
    double circumference = diameterOfWheel * Math.PI;
    int distanceToGo = 12;
    double rotations = distanceToGo / circumference;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DcMotor linearSlide_right;
    private DcMotor linearSlide_left;

    public CRServo In_Right;
    public CRServo In_Left;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18;// Tag ID 18 from the 36h11 family
    int RIGHT = 1;
    int MIDDLE = 2;
    int LEFT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Front_right = hardwareMap.dcMotor.get("FR");
        Front_left = hardwareMap.dcMotor.get("FL");
        backRight = hardwareMap.dcMotor.get("BR");
        backLeft = hardwareMap.dcMotor.get("BL");


        //motors mirror each other

        backRight.setDirection(DcMotor.Direction.REVERSE);
        Front_right.setDirection(DcMotor.Direction.REVERSE);


        //encoders
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlide_right = hardwareMap.get(DcMotor.class, "LR");
        linearSlide_left = hardwareMap.get(DcMotor.class, "LL");

        In_Right = hardwareMap.get(CRServo.class, "IRS");
        In_Left = hardwareMap.get(CRServo.class, "ILS");


        linearSlide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Put initialization blocks here.
        linearSlide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_left.setDirection(DcMotorSimple.Direction.FORWARD);

        In_Right.setDirection(CRServo.Direction.REVERSE);
        In_Left.setDirection(CRServo.Direction.FORWARD);

        linearSlide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide_right.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == RIGHT || tag.id == MIDDLE || tag.id == LEFT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }

        /***********************************************************
         * Actual CODE START HERE
         * *********************************************************
         *********************************************************
         *********************************************************
         *********************************************************
         *********************************************************
         *********************************************************
         *********************************************************
         *
         */

        if (tagOfInterest.id == LEFT) {

            DriveForward(.4, 125);
            StopDriving();
            sleep(1000);
            DriveBackwards(.4, 20);
            StopDriving();
            sleep(500);
            StrafeLeft(.4, 568);
            StopDriving();
            LIFT_POWER(-1);
            sleep(3000);
            DriveForward(.1, 2);
            LIFT_POWER(0);
            sleep(1500);
            In_Left.setPower(-1);
            In_Right.setPower(-1);
            sleep(1500);
            In_Right.setPower(0);
            In_Left.setPower(0);
            LIFT_POWER(-1);
            sleep(2000);
            DriveBackwards(.4, 4);
            StopDriving();
            linearSlide_left.setPower(0);
            linearSlide_right.setPower(0);
            StrafeRight(.4, 1850);
            DriveBackwards(.4, 10);
            StopDriving();


        } else if (tagOfInterest.id == MIDDLE) {
            DriveForward(.4, 125);
            StopDriving();
            sleep(1000);
            DriveBackwards(.4, 20);
            StopDriving();
            sleep(500);
            StrafeLeft(.4, 558);
            StopDriving();
            LIFT_POWER(-1);
            sleep(3000);
            DriveForward(.1, 2);
            LIFT_POWER(0);
            sleep(1500);
            In_Left.setPower(-1);
            In_Right.setPower(-1);
            sleep(1500);
            In_Right.setPower(0);
            In_Left.setPower(0);
            LIFT_POWER(-1);
            sleep(2000);
            DriveBackwards(.4, 4);
            StopDriving();
            linearSlide_left.setPower(0);
            linearSlide_right.setPower(0);
            StrafeRight(.4, 760);
            StopDriving();
            DriveBackwards(.4, 10);
            StopDriving();


        } else if (tagOfInterest.id == RIGHT) {
            DriveForward(.4, 125);
            StopDriving();
            sleep(1000);
            DriveBackwards(.4, 20);
            StopDriving();
            sleep(500);
            StrafeLeft(.4, 568);
            StopDriving();
            LIFT_POWER(-1);
            sleep(3000);
            DriveForward(.1, 2);
            LIFT_POWER(0);
            sleep(1500);
            In_Left.setPower(-1);
            In_Right.setPower(-1);
            sleep(1500);
            In_Right.setPower(0);
            In_Left.setPower(0);
            LIFT_POWER(-1);
            sleep(2000);
            DriveBackwards(.4, 4);
            StopDriving();
            linearSlide_left.setPower(0);
            linearSlide_right.setPower(0);
            StrafeLeft(.4, 770);
            DriveBackwards(.4, 10);


        } else {
            telemetry.addLine("(ERROR OCCUR)");
        }
    }


    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void LIFT_POWER(double power) {
        linearSlide_right.setPower(power);
        linearSlide_left.setPower(power);
    }

    public void DriveBackwards(double power, int distance) {

        int movecounts = (int) (distance * COUNTS_PER_INCH);

        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(-movecounts);
        Front_right.setTargetPosition(-movecounts);
        backRight.setTargetPosition(-movecounts);
        backLeft.setTargetPosition(-movecounts);

        //set mode
        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Driving Backwards", "Running to %7d", movecounts);
            telemetry.addData("Path2", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Path3", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Path4", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }
        StopDriving();
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward(double power, int distance) {

        //1240 = 1 rotation

        int movecounts = (int) (distance * COUNTS_PER_INCH);

        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(movecounts);
        Front_right.setTargetPosition(movecounts);
        backRight.setTargetPosition(movecounts);
        backLeft.setTargetPosition(movecounts);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);


        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Driving Forward", "Running to %7d", movecounts);
            telemetry.addData("Path2", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Path3", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Path4", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

        }
        StopDriving();

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void TurnRight(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(distance);
        Front_right.setTargetPosition(-distance);
        backRight.setTargetPosition(-distance);
        backLeft.setTargetPosition(distance);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Turning Right", "Running to %7d", distance);
            telemetry.addData("Path2", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Path3", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Path4", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        StopDriving();
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnLeft(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(-distance);
        Front_right.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Turning Left", "Running to %7d", distance);
            telemetry.addData("Front Left", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Front Right", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Back Right", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void StrafeRight(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(distance);
        Front_right.setTargetPosition(-distance);
        backRight.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Strafing Right", "Running to %7d", distance);
            telemetry.addData("Front Left", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Front Right", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Back Right", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void StrafeLeft(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(-distance);
        Front_right.setTargetPosition(distance);
        backRight.setTargetPosition(-distance);
        backLeft.setTargetPosition(distance);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Strafing Left", "Running to %7d", distance);
            telemetry.addData("Front Left", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Front Right", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Back Right", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Lift_Up(double power, int distance) {

        // reset encoders
        linearSlide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //target position
        linearSlide_right.setTargetPosition(distance);
        linearSlide_left.setTargetPosition(-distance);

        linearSlide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        linearSlide_right.setPower(power);
        linearSlide_left.setPower(power);
        while (opModeIsActive() && linearSlide_right.isBusy() && linearSlide_left.isBusy()) {
            telemetry.addData("Right Lift", "Running to %7d", linearSlide_right.getCurrentPosition());
            telemetry.addData("Left Lift", "Running to%7d", linearSlide_left.getCurrentPosition());
            telemetry.update();

            idle();
        }

        linearSlide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Lift_Down(double power, int distance) {

        // reset encoders
        linearSlide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //target position
        linearSlide_right.setTargetPosition(-180);
        linearSlide_left.setTargetPosition(180);

        linearSlide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        linearSlide_right.setPower(power);
        linearSlide_left.setPower(power);
        while (opModeIsActive() && linearSlide_right.isBusy() && linearSlide_left.isBusy()) {
            telemetry.addData("Right Lift", "Running to %7d", linearSlide_right.getCurrentPosition());
            telemetry.addData("Left Lift", "Running to%7d", linearSlide_left.getCurrentPosition());
            telemetry.update();

            idle();
        }

        linearSlide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void StopDriving() {
        Front_left.setPower(0);
        backLeft.setPower(0);
        Front_right.setPower(0);
        backRight.setPower(0);
        linearSlide_left.setPower(0);
        linearSlide_right.setPower(0);
    }
}