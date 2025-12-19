package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoCommon {
    private static DcMotor frontLeft;
    private static DcMotor frontRight;
    private static DcMotor backLeft;
    private static DcMotor backRight;

    private DcMotor launcherLeft;
    private DcMotor launcherRight;
    private DcMotor conveyorRight;
    private Servo gate;


    private enum LaunchState {
        IDLE,
        GATEOPEN,
        CONVEYOR,
        GATECLOSE,
        LAUNCHERS
    }

    private static LaunchState launchState;
    private static ElapsedTime spinTimer = new ElapsedTime();
    private static ElapsedTime shotTimer = new ElapsedTime();
    private static int shotCount = 0;

    private static ElapsedTime driveTimer = new ElapsedTime();

    public static void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        conveyorRight = hardwareMap.get(DcMotor.class, "conveyorRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        gate = hardwareMap.get(Servo.class, "gate");

        conveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);

        launchState = LaunchState.IDLE;

        driveTimer.reset();
        spinTimer.reset();
        shotTimer.reset();
    }

    public static boolean drive(double power, double seconds) {
        final double TOL_MM = 10;

        if (driveTimer.seconds() < seconds) {

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
            return false;
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return true;
        }
    }

    public static boolean strafe(double power, double seconds) {
        final double TOL_MM = 10;

        // If timer exceeded .25, we are done
        if (driveTimer.seconds() < seconds) {

            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
            return false;
        }

        // Stop motors
        else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            return true;
        }
    }

    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to create the
         * requested angle. We use radians here because it makes the math much easier.
         * Our robot will have rotated one radian when the wheels of the robot have driven
         * 1/2 of the track width of our robot in a circle. This is also the radius of the circle
         * that the robot tracks when it is rotating. So, to find the number of mm that our wheels
         * need to travel, we just need to multiply the requested angle in radians by the radius
         * of our turning circle.
         */
        double rx = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM/2);

        double leftTargetPosition = -(rx * TICKS_PER_MM);
//        double rightTargetPosition = targetMm * TICKS_PER_MM;
//
//        frontLeft.setTargetPosition((int) leftTargetPosition);
//        rightDrive.setTargetPosition((int) rightTargetPosition);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(1);
        frontRight.setPower(- 1);
        backLeft.setPower(1);
        backRight.setPower(- 1);

//        if ((Math.abs(leftTargetPosition - frontLeft.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
//            driveTimer.reset();
//        }
        if (driveTimer.seconds() > holdSeconds){
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        return (driveTimer.seconds() > holdSeconds);
    }

    public static boolean launch(boolean shotRequested, int count) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    shotCount = count;
                    launchState = LaunchState.LAUNCHERS;
                    spinTimer.reset();
                    shotTimer.reset();
                }
                break;

            case LAUNCHERS:
                launcherRight.setPower(.6);
                launcherLeft.setPower(.6);
                if(spinTimer.seconds() > 2) launchState = LaunchState.GATEOPEN;
                break;

            case GATEOPEN:
                gate.setPosition(.5);
                if (gate.getPosition() == (.5)) {
                    launchState = LaunchState.CONVEYOR;
                    shotTimer.reset();
                }
                break;

            case CONVEYOR:
                conveyorRight.setPower(1);
                if (shotTimer.seconds() > (0.75)) {
                    shotCount -= 1;
                    if(shotCount <= 0)
                        launchState = LaunchState.FINISH;
                }
                break;

            case FINISH:
                launcherLeft.setPower(0);
                launcherRight.setPower(0);
                conveyorRight.setPower(0);
                gate.setPosition(0);
                if(gate.getPosition() == 0){
                    launchState = LaunchState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }
}
