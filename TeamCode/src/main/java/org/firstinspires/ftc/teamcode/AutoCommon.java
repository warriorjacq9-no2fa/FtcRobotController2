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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    public enum SmartDriveState {
        IDLE,
        INIT,
        DRIVE,
        VERIFY,
        FINISH
    }

    public enum Alliance {
        BLUE,
        RED
    }

    // All states are public for telemetry usage
    public static SmartDriveState sDriveState;
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
        currentRX = au.toRadians(startRX);
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
            if(limelight != null) limelight.pipelineSwitch(0);

            conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
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
        sDriveState = SmartDriveState.IDLE;

        driveTimer.reset();
        spinTimer.reset();
        shotTimer.reset();
    }

    // Position is in meters, center origin.
    // For DECODE, +X is towards red goal, and +Y is away from goals
    public static Pose3D llPosition() {
        if(limelight == null) return null;
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
        if(limelight == null) return null;
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

    private static double dx, dy, drx;
    public static boolean drive
    (
        boolean start,
        double x, double y, double rx,
        double speed,
        DistanceUnit dUnit, AngleUnit aUnit,
        double holdSeconds
    ) {
        switch(sDriveState) {
            case IDLE:
                if(start) {
                    sDriveState = SmartDriveState.INIT;
                }
                break;

            case INIT:
                double g_dx = x - dUnit.fromMm(currentX);
                double g_dy = y - dUnit.fromMm(currentY);

                // Get robot-relative movement
                double cos = Math.cos(currentRX);
                double sin = Math.sin(currentRX);

                dx = cos * g_dx + sin * g_dy;
                dy = -sin * g_dx + cos * g_dy;
                sDriveState = SmartDriveState.DRIVE;
                break;

            case DRIVE:
                if(drive_rel(start, dx, dy, 0, speed, dUnit, aUnit, 0)) {
                    sDriveState = SmartDriveState.VERIFY;
                }
                break;

            case VERIFY:
                Pose3D pose = llPosition();
                if(pose == null) {
                    sDriveState = SmartDriveState.FINISH;
                    break;
                }
                Position pos = pose.getPosition();
                double yaw = pose.getOrientation().getYaw();
                dx = currentX - Math.round(pos.x);
                dy = currentY - Math.round(pos.y);
                drx = currentRX - Math.round(yaw);

                if(dx != 0 || dy != 0 || drx != 0)
                    sDriveState = SmartDriveState.DRIVE;
                else
                    sDriveState = SmartDriveState.FINISH;
                driveTimer.reset();
                break;

            case FINISH:
                if(driveTimer.seconds() > holdSeconds) {
                    sDriveState = SmartDriveState.IDLE;
                    return true;
                }
        }
        return false;
    }

    public static boolean drive_rel(
            boolean start,
            double dx, double dy, double drx,
            double speed,
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

                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);

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
                    currentX += x;
                    currentY += y;
                    currentRX += rx;
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
