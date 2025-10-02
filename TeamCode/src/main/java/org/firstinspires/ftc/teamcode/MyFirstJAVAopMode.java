package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp

public class MyFirstJAVAopMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    //private Servo servoTest;


    @Override
    public void runOpMode() {
      //  imu = hardwareMap.get(Gyroscope.class, "imu");
       frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
       // digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
       // sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
       // servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double y = 0; // Remember, Y stick is reversed!
        double x = 0;
        double rx = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            frontLeft.setPower(x + y + rx);
            frontRight.setPower(y - x -rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("rx", rx);
            telemetry.addData("Motor Power", frontLeft.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }
    }
}