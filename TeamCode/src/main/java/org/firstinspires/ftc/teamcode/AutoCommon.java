package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoCommon {
    private static DcMotor frontLeft;
    private static DcMotor frontRight;
    private static DcMotor backLeft;
    private static DcMotor backRight;

    private static ElapsedTime driveTimer = new ElapsedTime();

    public static void init(HardwareMap hardwareMap, String fl, String fr, String bl, String br) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);

        driveTimer.reset();
    }
    public static boolean drive(double power, double seconds) {
        final double TOL_MM = 10;

        if (driveTimer.seconds() < seconds) {

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

    public static boolean drive_side(double power, double seconds) {
        final double TOL_MM = 10;

        // If timer exceeded .25, we are done
        if (driveTimer.seconds() < seconds) {

            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
            return false;
        }

        // Stop motors
        else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            return true;
        }
    }
}
