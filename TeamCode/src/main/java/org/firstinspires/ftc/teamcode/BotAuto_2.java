package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BotAuto_2", group = "Bot")
public class BotAuto_2 extends OpMode {
    private enum AutonomousState {
        DRIVING,
        COMPLETE
    }

    private AutonomousState aState;

    @Override
    public void init() {
        AutoCommon.init(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight");

        aState = AutonomousState.DRIVING;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        switch (aState) {
            case DRIVING:
                if (AutoCommon.drive(1, 0.5)) {
                    //aState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("AutoState", aState);
    }
}