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
        GET_NEXT_ACTION,
        DRIVING,
        DRIVING_WAIT,
        LAUNCH,
        LAUNCH_WAIT,
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

    PathParser.DrivePath drivePath;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        autonomousState = AutonomousState.GET_NEXT_ACTION;

        AutoCommon.init(hardwareMap, telemetry, drivePath.origin.x, drivePath.origin.y, drivePath.origin.rx,
                drivePath.unit, AngleUnit.DEGREES);

        //intakeM = hardwareMap.get(DcMotor.class, "intake");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

        //telemetry.addData("Press X", "for BLUE");
        //telemetry.addData("Press B", "for RED");
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        telemetry.addData("Selected Alliance", alliance);
        drivePath = PathParser.parse(hardwareMap.appContext
                .getResources().getXml(R.xml.drivepath), alliance.name(), telemetry);
        AutoCommon.drive(false, 0, 0, 0, 1,DistanceUnit.INCH, AngleUnit.DEGREES, 1);
    }

    int actionIndex = 0;
    PathParser.PathAction action;
    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        AutoCommon.drive(false, 0, 0, 0, 1,DistanceUnit.INCH, AngleUnit.DEGREES, 1);
        /*
        PathParser.PointAction pointAction;
        PathParser.LaunchAction launchAction;
        switch (autonomousState) {
            case GET_NEXT_ACTION:
                if(actionIndex >= drivePath.actions.size()) {
                    autonomousState = AutonomousState.COMPLETE;
                    break;
                }

                action = drivePath.actions.get(actionIndex);
                if(action.getClass().equals(PathParser.PointAction.class)) {
                    autonomousState = AutonomousState.DRIVING;
                } else if(action.getClass().equals(PathParser.LaunchAction.class)) {
                    autonomousState = AutonomousState.LAUNCH;
                }
                actionIndex++;
                break;
            case DRIVING:
                pointAction = (PathParser.PointAction) action;
                if (AutoCommon.drive(true,
                        pointAction.x, pointAction.y, pointAction.rx,
                        pointAction.speed,
                        drivePath.unit, AngleUnit.DEGREES, 1)) {
                    autonomousState = AutonomousState.GET_NEXT_ACTION;
                } else autonomousState = AutonomousState.DRIVING_WAIT;
                break;

            case DRIVING_WAIT:
                pointAction = (PathParser.PointAction) action;
                if (AutoCommon.drive(false,
                        pointAction.x, pointAction.y, pointAction.rx,,
                        pointAction.speed,
                        drivePath.unit, AngleUnit.DEGREES, 1)) {
                    autonomousState = AutonomousState.GET_NEXT_ACTION;
                }
                break;

            case LAUNCH:
                launchAction = (PathParser.LaunchAction) action;
                if ((AutoCommon.launch(true, launchAction.count))) {
                    autonomousState = AutonomousState.GET_NEXT_ACTION;
                } else autonomousState = AutonomousState.LAUNCH_WAIT;
                break;

            case LAUNCH_WAIT:
                launchAction = (PathParser.LaunchAction) action;
                if(AutoCommon.launch(false, launchAction.count)) {
                    autonomousState = AutonomousState.GET_NEXT_ACTION;
                }
                break;
            case COMPLETE:
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", AutoCommon.launchState);
        telemetry.addData("DriveState", AutoCommon.driveState);
        telemetry.addData("Current action ID", action.id);
        telemetry.update();*/
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








