package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drivetrains.BasicDrivetrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto")
public class VisionTest extends LinearOpMode {
    public DcMotor Front_right;
    public DcMotor Back_right;
    public DcMotor Front_left;
    public DcMotor Back_left;
    public DcMotor Lift;
    public CRServo In_Right;
    public CRServo In_Left;

    SleeveDetection sleeveDetection = new SleeveDetection();
    BasicDrivetrain basicDrivetrain = new BasicDrivetrain();
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        Front_right = hardwareMap.get(DcMotor.class, "FR");
        Front_left = hardwareMap.get(DcMotor.class, "FL");
        Back_left = hardwareMap.get(DcMotor.class, "BL");
        Back_right = hardwareMap.get(DcMotor.class, "BR");
        In_Right = hardwareMap.get(CRServo.class, "IRS");
        In_Left = hardwareMap.get(CRServo.class, "ILS");
        Lift = hardwareMap.get(DcMotor.class, "L");

        Front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        In_Right.setDirection(CRServo.Direction.REVERSE);
        In_Left.setDirection(CRServo.Direction.FORWARD);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);


        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        waitForStart();
        while (opModeIsActive()) {
            if (position == SleeveDetection.ParkingPosition.CENTER) {
                Strafe_at_angle(1, 0);
                sleep(1000);
                Zero_power();
                return;
            }
            if (position == SleeveDetection.ParkingPosition.LEFT) {
                Strafe_at_angle(1, 270);
                sleep(1000);
                Zero_power();
                Strafe_at_angle(1, 180);
                sleep(500);
                Zero_power();
                Strafe_at_angle(1, 0);
                sleep(1000);
                Zero_power();
                return;
            }
            if (position == SleeveDetection.ParkingPosition.RIGHT) {
                Strafe_at_angle(1, 90);
                sleep(1650);
                Zero_power();
                Strafe_at_angle(1, 0);
                sleep(1000);
                Zero_power();
                return;
            }
        }
    }

    public void Strafe_at_angle(double Power, double Angle) {
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

    public void Zero_power() {
        Front_right.setPower(0);
        Back_right.setPower(0);
        Front_left.setPower(0);
        Back_left.setPower(0);
    }
}