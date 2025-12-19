package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        AutoCommon.init(hardwareMap);

        aState = AutonomousState.DRIVING;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        switch (aState) {
            case DRIVING:
                if (AutoCommon.drive(true, 0, 6, 0, DistanceUnit.INCHES, AngleUnit.DEGREES, 1)) {
                    aState = AutonomousState.COMPLETE;
                } else aState = AutonomousState.DRIVING_WAIT;
                break;

            case DRIVING_WAIT:
                if (AutoCommon.drive(false, 0, 6, 0, DistanceUnit.INCHES, AngleUnit.DEGREES, 1)) {
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