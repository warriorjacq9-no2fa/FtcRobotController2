package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AutoCommon {
    static final double WHEEL_DIAMETER_MM = 96;
    static final double ENCODER_TICKS_PER_REV = 537.7;
    static final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    static final double TURN_RADIUS_MM = 404;

    private static DcMotor frontLeft;
    private static DcMotor frontRight;
    private static DcMotor backLeft;
    private static DcMotor backRight;

    private static DcMotor launcherLeft;
    private static DcMotor launcherRight;
    private static DcMotor conveyorRight;
    private static Servo gate;

    private static IMU imu;
    private static Limelight3A limelight;


    public enum LaunchState {
        IDLE,
        LAUNCHERS,
        GATE_OPEN,
        CONVEYOR,
        FINISH
    }

    public enum DriveState {
        IDLE,
        RESET,
        INIT,
        DRIVE,
        FINISH
    }

    public enum Alliance {
        BLUE,
        RED
    }

    // All states are public for telemetry usage
    public static DriveState driveState;
    public static LaunchState launchState;

    private static final ElapsedTime spinTimer = new ElapsedTime();
    private static final ElapsedTime shotTimer = new ElapsedTime();
    private static int shotCount = 0;

    private static final ElapsedTime driveTimer = new ElapsedTime();

    private static double currentX = 0;
    private static double currentY = 0;
    private static double currentRX = 0;

    public static void init(HardwareMap hardwareMap, Telemetry telemetry,
                            double startX, double startY, double startRX,
                            DistanceUnit du, AngleUnit au) {
        currentX = du.toMm(startX);
        currentY = du.toMm(startY);
        currentRX = au.toDegrees(startRX);
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            conveyorRight = hardwareMap.get(DcMotor.class, "conveyorRight");
            launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
            launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
            gate = hardwareMap.get(Servo.class, "gate");

            imu = hardwareMap.get(IMU.class, "imu");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);

            conveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);
            gate.setDirection(Servo.Direction.REVERSE);

            frontLeft.setZeroPowerBehavior(BRAKE);
            frontRight.setZeroPowerBehavior(BRAKE);
            backRight.setZeroPowerBehavior(BRAKE);
            backLeft.setZeroPowerBehavior(BRAKE);
        } catch (IllegalArgumentException e) {
            telemetry.addData("Error during setup", e.getMessage());
            telemetry.speak(e.getMessage());
        }

        launchState = LaunchState.IDLE;
        driveState = DriveState.IDLE;

        driveTimer.reset();
        spinTimer.reset();
        shotTimer.reset();
    }

    // Position is in meters, center origin.
    // For DECODE, +X is towards red goal, and +Y is away from goals
    public static Pose3D llPosition() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getBotpose_MT2();
            }
        }
        return null;
    }

    public static double[] llTarget(Alliance alliance) {
        limelight.pipelineSwitch(alliance.ordinal() + 1);
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            if(result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                return new double[] {tx, ty, ta};
            }
        }
        return null;
    }

    public static boolean drive
    (
        boolean start,
        double x, double y, double rx,
        DistanceUnit dUnit, AngleUnit aUnit,
        double holdSeconds
    ) {
        double g_dx = dUnit.fromMm(currentX) - x;
        double g_dy = dUnit.fromMm(currentY) - y;

        // Get robot-relative movement
        double cos = Math.cos(currentRX);
        double sin = Math.sin(currentRX);
        
        double dx = cos * g_dx + sin * g_dy;
        double dy = -sin * g_dx + cos * g_dy;

        currentX = x;
        currentY = y;
        currentRX += aUnit.toDegrees(rx);
        return drive_rel(start, dx, dy, 0, dUnit, aUnit, holdSeconds);
    }

    private static boolean drive_rel(
            boolean start,
            double dx, double dy, double drx,
            DistanceUnit dUnit, AngleUnit aUnit,
            double holdSeconds
    ) {
        double x = (dUnit.toMm(dx)) * TICKS_PER_MM;
        double y = (dUnit.toMm(dy)) * TICKS_PER_MM;
        double rx = ((aUnit.toRadians(drx)) * TURN_RADIUS_MM) * TICKS_PER_MM;

        switch(driveState) {
            case IDLE:
                if(start) {
                    driveState = DriveState.RESET;
                }
                break;

            case RESET:
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                driveState = DriveState.INIT;
                break;

            case INIT:
                frontLeft.setTargetPosition((int) (y + x + rx));
                frontRight.setTargetPosition((int) (y - x - rx));
                backLeft.setTargetPosition((int) (y - x + rx));
                backRight.setTargetPosition((int) (y + x - rx));

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveState = DriveState.DRIVE;
                break;

            case DRIVE:
                if (!(frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {
                    driveTimer.reset();
                    driveState = DriveState.FINISH;
                }
                break;

            case FINISH:
                if(driveTimer.seconds() > holdSeconds) {
                    driveState = DriveState.IDLE;
                    return true;
                }
                break;
        }

        return false;
    }

    public static boolean launch(boolean shotRequested, int count) {
        switch (launchState) {
            case IDLE:
                gate.setPosition(0);
                if (shotRequested) {
                    shotCount = count;
                    launchState = LaunchState.LAUNCHERS;
                    spinTimer.reset();
                }
                break;

            case LAUNCHERS:
                launcherRight.setPower(.6);
                launcherLeft.setPower(.6);
                if(spinTimer.seconds() > 2) launchState = LaunchState.GATE_OPEN;
                break;

            case GATE_OPEN:
                gate.setPosition(.5);
                if (gate.getPosition() == (.5)) {
                    launchState = LaunchState.CONVEYOR;
                    shotTimer.reset();
                }
                break;

            case CONVEYOR:
                conveyorRight.setPower(1);
                if (shotTimer.seconds() > (0.75)) {
                    shotTimer.reset();
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
