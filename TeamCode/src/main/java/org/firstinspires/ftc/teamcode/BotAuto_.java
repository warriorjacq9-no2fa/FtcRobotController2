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
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Declare OpMode members.
    private Gyroscope imu;
    //private DcMotor intakeM;

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

    private Alliance alliance = Alliance.RED;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;

        AutoCommon.init(hardwareMap);

        //intakeM = hardwareMap.get(DcMotor.class, "intake");

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
                if ((AutoCommon.launch(true, 3))) {
                    autonomousState = AutonomousState.DRIVING_TOWARDS_GOAL;
                    driveTimer.reset();
                    launcherLeft.setPower(0);
                    launcherRight.setPower(0);
                    conveyorRight.setPower(0);
                } else autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if(AutoCommon.launch(false, 3)) {
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
                if (AutoCommon.strafe(1, 0.75)) {
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








