package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BotAuto_", group = "Bot")
public class BotAuto_ extends OpMode {
    final double FEED_TIME = 0.20; //The feeder servos run this long when a shot is requested.

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    /*
     * The number of seconds that we wait between each of our 3 shots from the launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes the likelihood
     * that each shot will score.
     */
    final double TIME_BETWEEN_SHOTS = 2;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either side of the
     * robot. Track width is used to determine the amount of linear distance each wheel needs to
     * travel to create a specified rotation of the robot.
     */
    final double DRIVE_SPEED = 0;
    final double ROTATE_SPEED = 0;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3; //The number of shots to fire in this auto.

    double robotRotationAngle = 45;

    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private ElapsedTime shotTimer = new ElapsedTime();
    //private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();

    // Declare OpMode members.
    private Gyroscope imu;
    private DcMotor launcherLeft;
    private DcMotor launcherRight;
    //private DcMotor intakeM;
    private DcMotor conveyorRight;
    private Servo gate;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "LaunchState." It reflects the current condition of the shooter motor when we request a shot.
     * It starts at IDLE. When a shot is requested from the user, it'll move into PREPARE then LAUNCH.
     * We can use higher level code to cycle through these states, but this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits."
     */
    private enum LaunchState {
        IDLE,
        GATEOPEN,
        CONVEYOR,
        GATECLOSE,
        LAUNCHERS
    }

    /*
     * Here we create the instance of LaunchState that we use in code. This creates a unique object
     * which can store the current condition of the shooter. In other applications, you may have
     * multiple copies of the same enum which have different names. Here we just have one.
     */
    private LaunchState launchState;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        RESET_AFTER_DRIVE,
        WAIT_FOR_LAUNCH,
        DRIVING_TOWARDS_GOAL,
        DRIVING_OFF_LINE,
        COMPLETE
    }

    private AutonomousState autonomousState;

    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
        launchState = LaunchState.IDLE;


        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */

        AutoCommon.init(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight");

        //intakeM = hardwareMap.get(DcMotor.class, "intake");
        conveyorRight = hardwareMap.get(DcMotor.class, "conveyorRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        gate = hardwareMap.get(Servo.class, "gate");

        conveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {

        conveyorRight.setPower(0);


        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        gate.setPosition(0);
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState) {
            /*
             * Since the first state of our auto is LAUNCH, this is the first "case" we encounter.
             * This case is very simple. We call our .launch() function with "true" in the parameter.
             * This "true" value informs our launch function that we'd like to start the process of
             * firing a shot. We will call this function with a "false" in the next case. This
             * "false" condition means that we are continuing to call the function every loop,
             * allowing it to cycle through and continue the process of launching the first ball.
             */


            case DRIVING_AWAY_FROM_GOAL:
                if (AutoCommon.drive(1, 1)) {
                    stopAllDrive();
                    autonomousState = AutonomousState.RESET_AFTER_DRIVE;
                }
                break;

            case RESET_AFTER_DRIVE:
                resetAllDriveEncoders();
                autonomousState = AutonomousState.LAUNCH;
                break;

            case LAUNCH:
                if ((launch(true, 3))) {
                    autonomousState = AutonomousState.DRIVING_TOWARDS_GOAL;
                    driveTimer.reset();
                    launcherLeft.setPower(0);
                    launcherRight.setPower(0);
                    conveyorRight.setPower(0);
                } else autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if(launch(false, 3)) {
                    autonomousState = AutonomousState.DRIVING_TOWARDS_GOAL;
                    driveTimer.reset();
                    launcherLeft.setPower(0);
                    launcherRight.setPower(0);
                    conveyorRight.setPower(0);
                }
                break;
            case DRIVING_TOWARDS_GOAL:
                if(AutoCommon.drive(-1, 1)) {
                    stopAllDrive();
                    driveTimer.reset();
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                telemetry.addData("Driving", "Off line");
                telemetry.addData("Drive timer", driveTimer.seconds());
                telemetry.update();
                if (AutoCommon.drive_side(1, 0.75)) {
                    stopAllDrive();
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Gate Posisition", gate.getPosition());
        telemetry.update();
    }

    private void stopAllDrive() {
        driveTimer.reset();
    }

    private void resetAllDriveEncoders() {
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    private ElapsedTime spinTimer = new ElapsedTime();
    static int shotCount = 0;
    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     *
     * @param shotRequested "true" if the user would like to fire a new shot, and "false" if a shot
     *                      has already been requested and we need to continue to move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched, "false" otherwise.
     */
    boolean launch(boolean shotRequested, int count) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    shotCount = count;
                    launchState = LaunchState.LAUNCHERS;
                    spinTimer.reset();
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
                        launchState = LaunchState.GATECLOSE;
                }
                break;

            case GATECLOSE:
                gate.setPosition(0);
                if(gate.getPosition() == 0){
                    launchState = LaunchState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }


    /**
     * @param speed       From 0-1
     * @param angle       the amount that the robot should rotate
     * @param angleUnit   the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the motors are within tolerance of the target position for more than
     * holdSeconds. False otherwise.
     */
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
}


//private DigitalChannel digitalTouch;
//private DistanceSensor sensorColorRange;
//private Limelight3A limelight;

//private Servo servoTest;
//    public void limelight() {
//        LLResult result = limelight.getLatestResult();
//        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//        for (LLResultTypes.FiducialResult fiducial : fiducials) {
//            int id = fiducial.getFiducialId(); // The ID number of the fiducial
//            double xdegrees = fiducial.getTargetXDegrees(); // Where it is (left-right)
//            double ydegrees = fiducial.getTargetYDegrees(); // Where it is (up-down)
//            double distance = fiducial.getTargetPoseCameraSpace().getPosition().z;
//            telemetry.addData("Fiducial ", id);
//            telemetry.addData("xdegrees", xdegrees);
//            telemetry.addData("ydegrees", ydegrees);
//            telemetry.addData("distance", distance);








