package org.firstinspires.ftc.teamcode.Autonomous.Color_Detection;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * A simple test of  {@link DuckPipeline}
 */
@Disabled
@Autonomous(name = "OpenCV Blue", group = "Tutorials")
public class BlueDuck extends LinearOpMode {
    int colorArea1;
    int colorArea2;
    public static final double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static final double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static final double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static final double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    // Yellow Range                                      Y      Cr     Cb
    public Scalar scalarLowerYCrCb = new Scalar(0.0, 0, 0);
    public Scalar scalarUpperYCrCb = new Scalar(225, 120, 120);
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private OpenCvCamera webcam;
    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;
    private double lowerruntime = 0;
    private double upperruntime = 0;
    private DcMotor Front_right;
    private DcMotor Back_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor linearslide;
    private CRServo Right_Servo;
    private CRServo Left_Servo;

    @Override
    public void runOpMode() throws InterruptedException {
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        linearslide = hardwareMap.get(DcMotor.class, "linearslide");
        Right_Servo = hardwareMap.get(CRServo.class, "Right_Servo");
        Left_Servo = hardwareMap.get(CRServo.class, "Left_Servo");
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        DuckPipeline myPipeline;
        webcam.setPipeline(myPipeline = new DuckPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        for (int i = 0; i < 5; i++) {
            myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
            myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
            sleep(1000);
            colorArea1 = (int) myPipeline.getRectArea();

            myPipeline.Green = new Scalar(196, 23, 112);
            myPipeline.scalarLowerYCrCb = new Scalar(0, 255, 200);
            myPipeline.scalarUpperYCrCb = new Scalar(255, 140, 255);
            scalarLowerYCrCb = new Scalar(0, 255, 200);
            scalarUpperYCrCb = new Scalar(255, 140, 255);

            myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
            myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
            sleep(1000);
            colorArea2 = (int) myPipeline.getRectArea();
        }

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        while (!isStarted()) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
        }
        Inti();
        waitForStart();

        while (opModeIsActive()) {

            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if (colorArea2 > colorArea1) {
                {
                    Strafe_at_angle(0.5, 0);
                    sleep(1500);
                    Zero_power();
                    return;
                }
            } else if (colorArea1 > colorArea2) {
                Strafe_at_angle(0.5, 90);
                sleep(1650);
                Zero_power();
                Strafe_at_angle(0.5, 0);
                sleep(1500);
                Zero_power();
                return;
            } else {
                Strafe_at_angle(0.5, 180);
                sleep(1650);
                Zero_power();
                Strafe_at_angle(0.5, 0);
                sleep(1500);
                Zero_power();
                return;
            }
        }
    }

    public void testing(DuckPipeline myPipeline) {
        if (lowerruntime + 0.05 < getRuntime()) {
            CrLowerUpdate -= gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if (upperruntime + 0.05 < getRuntime()) {
            CrUpperUpdate -= gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int) CrLowerUpdate);
        telemetry.addData("lowerCb ", (int) CbLowerUpdate);
        telemetry.addData("UpperCr ", (int) CrUpperUpdate);
        telemetry.addData("UpperCb ", (int) CbUpperUpdate);
    }

    /**
     * Thresholds value between min and max
     *
     * @param value The value to be thresholded
     * @param min   The minimum acceptable value
     * @param max   The maximum acceptable value
     * @return Value if it is between min and max, max if value > max, or min if value < min
     */
    private double inValues(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    /**
     * A dummy autonomous to demonstrate that path A was taken
     */
    private void AUTONOMOUS_A() {
        telemetry.addLine("Autonomous A");
    }

    /**
     * A dummy autonomous to demonstrate that path B was taken
     */
    private void AUTONOMOUS_B() {
        telemetry.addLine("Autonomous B");
    }

    /**
     * A dummy autonomous to demonstrate that path C was taken
     */
    private void AUTONOMOUS_C() {
        telemetry.addLine("Autonomous C");
    }

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
    }
}
