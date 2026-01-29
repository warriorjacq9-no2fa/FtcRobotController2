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
    static final double WHEEL_DIAMETER_MM = 104;
    static final double ENCODER_TICKS_PER_REV = 751.8;
    static final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    static final double TURN_RADIUS_MM = 250;

    static final double TOLERANCE_MM = 10;
    static final double TOLERANCE_TICKS = TOLERANCE_MM * TICKS_PER_MM;
    static final double TOLERANCE_DEG = 5;

    static final double LL_SAMPLE_SIZE = 5;

    private static DcMotor frontLeft;
    private static DcMotor frontRight;
    private static DcMotor backLeft;
    private static DcMotor backRight;

    private static DcMotor launcherRight;
    private static DcMotor conveyorRight;
    private static Servo gate;

    private static IMU imu;
    private static Limelight3A limelight;

    public static class Pos3d {
        public double x, y, a;

        public Pos3d() {
            x = 0;
            y = 0;
            a = 0;
        }

        public Pos3d(double x, double y, double a) {
            this.x = x;
            this.y = y;
            this.a = a;
        }
    }

    public enum LaunchState {
        IDLE,
        LAUNCHERS,
        GATE_OPEN,
        CONVEYOR,
        GATE_CLOSE,
        FINISH
    }

    public enum DriveState {
        IDLE,
        RESET,
        INIT_DRIVE,
        DRIVE,
        INIT_ROTATE,
        ROTATE,
        FINISH
    }

    public enum SmartDriveState {
        IDLE,
        LIMELIGHT,
        DRIVE,
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

    private static Telemetry telemetry;

    public static void init(HardwareMap hardwareMap, Telemetry telemetry) {
        AutoCommon.telemetry = telemetry;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        conveyorRight = hardwareMap.get(DcMotor.class, "conveyorRight");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        gate = hardwareMap.get(Servo.class, "gate");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);

        launchState = LaunchState.IDLE;
        driveState = DriveState.IDLE;
        sDriveState = SmartDriveState.IDLE;
    }

    public static void start() {
        driveTimer.reset();
        spinTimer.reset();
        shotTimer.reset();
    }

    // Position is in MM, center origin.
    // For DECODE, +X is towards red goal, and +Y is away from goals
    public static Pos3d llPosition() {
        if(limelight == null) return null;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        Pos3d res = new Pos3d();
        int j = 0;
        for(int i = 0; i < LL_SAMPLE_SIZE; i++) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D pose = result.getBotpose_MT2();
                    res.x += pose.getPosition().unit.toMm(pose.getPosition().x);
                    res.y += pose.getPosition().unit.toMm(pose.getPosition().y);
                    res.a += pose.getOrientation().getYaw();
                    j++;
                }
            }
        }
        res.x /= j;
        res.y /= j;
        res.a /= j;
        return res;
    }

    public static Pos3d llTarget(Alliance alliance) {
        if(limelight == null) return null;
        limelight.pipelineSwitch(alliance.ordinal() + 1);
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            if(result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                return new Pos3d(tx, ty, ta);
            }
        }
        return null;
    }


    private static double fl, fr, bl, br;
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
                    sDriveState = SmartDriveState.LIMELIGHT;
                }
                break;
            case LIMELIGHT:
                Pos3d pose = llPosition();
                if(pose != null) {
                    double g_dx = dUnit.toMm(x) - Math.round(pose.x);
                    double g_dy = dUnit.toMm(y) - Math.round(pose.y);
                    double g_drx = pose.a * (Math.PI / 180); // Convert to radians

                    double distance = Math.hypot(g_dx, g_dy);

                    double f_vx = 0.0;
                    double f_vy = 0.0;

                    if (distance > TOLERANCE_MM) {
                        f_vx = (g_dx / distance) * speed;
                        f_vy = (g_dy / distance) * speed;
                    }

                    // Get robot-relative movement
                    double cos = Math.cos(g_drx);
                    double sin = Math.sin(g_drx);

                    double vx = f_vx * cos + f_vy * sin;
                    double vy = -f_vx * sin + f_vy * cos;

                    double omega = aUnit.toRadians(rx) - (pose.a * (Math.PI / 180));

                    fl = vx + vy + omega;
                    fr = vx - vy - omega;
                    bl = vx - vy + omega;
                    br = vx + vy - omega;

                    double max = Math.max(
                            Math.max(Math.abs(fl), Math.abs(fr)),
                            Math.max(Math.abs(bl), Math.abs(br))
                    );

                    if (max > 1.0) {
                        fl /= max;
                        fr /= max;
                        bl /= max;
                        br /= max;
                    }

                    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sDriveState = SmartDriveState.DRIVE;
                }
                break;
            
            case DRIVE:
                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);
                sDriveState = SmartDriveState.LIMELIGHT;
                break;
            
            case FINISH:
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                if(driveTimer.seconds() > holdSeconds) {
                    sDriveState = SmartDriveState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }

    private static boolean isBusy(DcMotor m) {
        if(m == null) return false;
        if(m.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        return Math.abs(m.getTargetPosition() - m.getCurrentPosition()) >= TOLERANCE_TICKS;
    }

    public static boolean drive_rel
    (
        boolean start,
        double dx, double dy, double drx,
        double speed,
        DistanceUnit dUnit, AngleUnit aUnit,
        double holdSeconds
    ) {
        double x = (dUnit.toMm(dx)) * TICKS_PER_MM;
        double y = (dUnit.toMm(dy)) * TICKS_PER_MM;
        double rx = (((aUnit.toRadians(drx)) * TURN_RADIUS_MM) / (Math.PI * WHEEL_DIAMETER_MM)) * TICKS_PER_MM;

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

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);

                driveState = DriveState.INIT_DRIVE;
                break;

            case INIT_DRIVE:
                frontLeft.setTargetPosition((int) (y + x));
                frontRight.setTargetPosition((int) (y - x));
                backLeft.setTargetPosition((int) (y - x));
                backRight.setTargetPosition((int) (y + x));

                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                driveState = DriveState.DRIVE;
                break;

            case DRIVE:
                if (!(isBusy(frontLeft) || isBusy(frontRight) ||
                        isBusy(backLeft) || isBusy(backRight))) {
                    driveTimer.reset();
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);

                    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    backRight.setDirection(DcMotorSimple.Direction.REVERSE);

                    if(aUnit.toDegrees(Math.abs(drx)) < TOLERANCE_DEG) {
                        driveState = DriveState.FINISH;
                        break;
                    }
                    driveState = DriveState.INIT_ROTATE;
                }
                break;
            
            case INIT_ROTATE:
                frontLeft.setTargetPosition((int) (-rx));
                frontRight.setTargetPosition((int) (rx));
                backLeft.setTargetPosition((int) (-rx));
                backRight.setTargetPosition((int) (rx));

                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                driveState = DriveState.ROTATE;
                break;
            
            case ROTATE:
                if (!(isBusy(frontLeft) || isBusy(frontRight) ||
                        isBusy(backLeft) || isBusy(backRight))) {
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
                gate.setPosition(1);
                if (shotRequested) {
                    shotCount = count;
                    launchState = LaunchState.LAUNCHERS;
                    spinTimer.reset();
                }
                break;

            case LAUNCHERS:
                launcherRight.setPower(1);
                if(spinTimer.seconds() > 2) launchState = LaunchState.GATE_OPEN;
                shotTimer.reset();
                break;

            case GATE_OPEN:
                gate.setPosition(0.5);
                if (shotTimer.seconds() > 0.25) {
                    launchState = LaunchState.CONVEYOR;
                    shotTimer.reset();
                }
                break;

            case CONVEYOR:
                conveyorRight.setPower(1);
                if (shotTimer.seconds() > 0.75) {
                    shotTimer.reset();
                    shotCount -= 1;
                    if(shotCount <= 0)
                        launchState = LaunchState.FINISH;
                    else
                        launchState = LaunchState.GATE_CLOSE;
                }
                break;
            
            case GATE_CLOSE:
                conveyorRight.setPower(0);
                gate.setPosition(1);
                if(shotTimer.seconds() > 0.25) {
                    launchState = LaunchState.GATE_OPEN;
                    shotTimer.reset();
                }
                break;

            case FINISH:
                launcherRight.setPower(0);
                conveyorRight.setPower(0);
                gate.setPosition(1);
                launchState = LaunchState.IDLE;
                return true;
        }
        return false;
    }
}
