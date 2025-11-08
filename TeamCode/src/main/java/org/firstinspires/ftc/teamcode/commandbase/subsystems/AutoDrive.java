package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoDrive {
    private final DcMotorEx FLmotor, FRmotor, BLmotor, BRmotor;

    public AutoDrive(HardwareMap hwMap) {
        FLmotor = hwMap.get(DcMotorEx.class, "FLmotor");
        FRmotor = hwMap.get(DcMotorEx.class, "FRmotor");
        BLmotor = hwMap.get(DcMotorEx.class, "BLmotor");
        BRmotor = hwMap.get(DcMotorEx.class, "BRmotor");

        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Directions for OUTWARD mecanum rollers (diamond pattern)
        FLmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drivewithpower(double power) {
        FLmotor.setPower(1.0);
        FRmotor.setPower(1.0);
        BLmotor.setPower(1.0);
        BRmotor.setPower(1.0);
    }
    public void stopauto() {
        FLmotor.setPower(0.0);
        FRmotor.setPower(0.0);
        BLmotor.setPower(0.0);
        BRmotor.setPower(0.0);
    }
}