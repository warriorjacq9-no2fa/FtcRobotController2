package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BotAuto_2", group = "Bot")
public class BotAuto_2 extends OpMode {
    private enum AutonomousState {
        DRIVING,
        DRIVING_WAIT,
        COMPLETE
    }
    private AutonomousState aState;

    @Override
    public void init() {
        AutoCommon.init(hardwareMap, telemetry);

        aState = AutonomousState.DRIVING;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        AutoCommon.start(0, 0, 0,DistanceUnit.INCH, AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        switch (aState) {
            case DRIVING:
                if (AutoCommon.drive_rel(true, 0, 12, 0, 1, DistanceUnit.INCH, AngleUnit.DEGREES, 1)) {
                    aState = AutonomousState.COMPLETE;
                } else aState = AutonomousState.DRIVING_WAIT;
                break;

            case DRIVING_WAIT:
                if (AutoCommon.drive_rel(false, 0, 12, 0, 1, DistanceUnit.INCH, AngleUnit.DEGREES, 1)) {
                    aState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;

        }
        telemetry.addData("AutoState", aState);
        telemetry.addData("LauncherState", AutoCommon.launchState);
        telemetry.addData("DriveState", AutoCommon.driveState);
    }
}