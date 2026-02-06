package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoCommon.drive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp

public class MainBotTeleOp extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor launcherRight;
    private DcMotor launcherLeft;
    private DcMotor intakeM;
    private CRServo conveyorRight;
    private CRServo conveyorLeft;
    private DcMotor lift;

    private Servo gate;
    private Servo pushdown1;
    private Servo pushdown2;
    private Servo pushdown3;
    private Servo pushdown4;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Limelight3A limelight;

    //private Servo servoTest;
    public void limelight() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double xdegrees = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double ydegrees = fiducial.getTargetYDegrees(); // Where it is (up-down)
            double distance = fiducial.getTargetPoseCameraSpace().getPosition().z;
            telemetry.addData("Fiducial ", id);
            telemetry.addData("xdegrees", xdegrees);
            telemetry.addData("ydegrees", ydegrees);
            telemetry.addData("distance", distance);

        }
    }

    public void launcher() {
        double power = (Math.log(gamepad2.left_stick_y) / Math.PI) + 1;
        launcherRight.setPower(power);
        launcherLeft.setPower(power);
        telemetry.addData("Launcher", launcherRight.getPower());
    }

    public void intake() {
        if(gamepad1.a) {
            intakeM.setPower(1);
        } else if(gamepad1.b) {
            intakeM.setPower(0);
        }
        telemetry.addData("Intake Power", intakeM.getPower());
    }
    public void gate() {
        if(gamepad2.right_bumper) {
            gate.setPosition(.5);
            pushdown1.setPosition(1);
            pushdown2.setPosition(1);
            pushdown3.setPosition(1);
            pushdown4.setPosition(1);

        } else {
            gate.setPosition(1);
        }
    }

    public void conveyor() {
        if (gamepad2.left_bumper) {
            conveyorRight.setPower(1);
            conveyorLeft.setPower(1);
        } else {
            conveyorRight.setPower(0);
            conveyorLeft.setPower(0);
        }
        telemetry.addData("conveyor", conveyorRight.getPower());



    }

    public void drive() {
        double y = 0; // Remember, Y stick is reversed!
        double x = 0;
        double rx = 0;
        // run until the end of the match (driver presses STOP)

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        frontLeft.setPower(y + x - rx);
        frontRight.setPower(y - x + rx);
        backLeft.setPower(y - x - rx);
        backRight.setPower(y + x + rx);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        telemetry.addData("Motor Power", frontLeft.getPower());
        telemetry.addData("Status", "Running");
    }

    public void lifts() {
        if(gamepad1.left_bumper) {
            lift.setPower(1);
        } else {
            lift.setPower(0);
        }
    }


    @Override
    public void runOpMode() {

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();
        intakeM = hardwareMap.get(DcMotor.class, "intake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        conveyorRight = hardwareMap.get(CRServo.class, "conveyorRight");
        conveyorLeft = hardwareMap.get(CRServo.class, "conveyorLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        gate = hardwareMap.get(Servo.class, "gate");
        pushdown1 = hardwareMap.get(Servo.class, "pushdown1");
        pushdown2 = hardwareMap.get(Servo.class, "pushdown2");
        pushdown3 = hardwareMap.get(Servo.class, "pushdown3");
        pushdown4 = hardwareMap.get(Servo.class, "pushdown4");
        lift = hardwareMap.get(DcMotor.class, "lift");

        conveyorRight.setPower(0);
        //launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            drive();
            //limelight();
            intake();
            gate();
            conveyor();
            launcher();
            lifts();
        }
        telemetry.update();
    }
}

//testing masters