package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BotAuto_", group = "Bot")
public class BotAuto_ extends OpMode {
    // Declare OpMode members.
    //private DcMotor intakeM;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        DRIVING_AWAY,
        DRIVING_AWAY_WAIT,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_OFF,
        DRIVING_OFF_WAIT,
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
        autonomousState = AutonomousState.DRIVING_AWAY;

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
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        telemetry.addData("Selected Alliance", alliance);
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        switch (autonomousState) {
            case DRIVING_AWAY:
                if (AutoCommon.drive(true, 0, 18, 0, DistanceUnit.INCH, AngleUnit.DEGREES, 1)) {
                    autonomousState = AutonomousState.LAUNCH;
                } else autonomousState = AutonomousState.DRIVING_AWAY_WAIT;
                break;

            case DRIVING_AWAY_WAIT:
                if (AutoCommon.drive(false, 0, 18, 0, DistanceUnit.INCH, AngleUnit.DEGREES, 1)) {
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;

            case LAUNCH:
                if ((AutoCommon.launch(true, 3))) {
                    autonomousState = AutonomousState.DRIVING_OFF;
                } else autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if(AutoCommon.launch(false, 3)) {
                    autonomousState = AutonomousState.DRIVING_OFF;
                }
                break;
            case DRIVING_OFF:
                if(AutoCommon.drive(true, 6, 12, 0, DistanceUnit.INCH, AngleUnit.DEGREES, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                } else autonomousState = AutonomousState.DRIVING_OFF_WAIT;
                break;

            case DRIVING_OFF_WAIT:
                if(AutoCommon.drive(false, 6, 12, 0, DistanceUnit.INCH, AngleUnit.DEGREES, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                } 
                break;

            case COMPLETE:
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", AutoCommon.launchState);
        telemetry.addData("DriveState", AutoCommon.driveState);
        telemetry.update();
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








