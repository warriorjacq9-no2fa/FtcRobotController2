package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BotAuto_2", group = "Bot")
public class BotAuto_2 extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private ElapsedTime driveTimer;

    private enum AutonomousState {
        DRIVING,
        COMPLETE
    }

    private AutonomousState aState;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);

        driveTimer.reset();

        aState = AutonomousState.DRIVING;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        switch(aState) {
            case DRIVING:
                if(drive(1)) {
                    aState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("AutoState", aState);
    }

    boolean drive(double power) {
        final double TOL_MM = 10;

        if (driveTimer.seconds() < 0.5) {

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
            return false;
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return true;
        }
    }
}