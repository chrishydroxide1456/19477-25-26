package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class savedDrive {
    private final DcMotorEx FLmotor, FRmotor, BLmotor, BRmotor;

    public savedDrive(HardwareMap hwMap) {
        FLmotor = hwMap.get(DcMotorEx.class, "FLmotor");
        FRmotor = hwMap.get(DcMotorEx.class, "FRmotor");
        BLmotor = hwMap.get(DcMotorEx.class, "BLmotor");
        BRmotor = hwMap.get(DcMotorEx.class, "BRmotor");

        // Directions for OUTWARD mecanum rollers (diamond pattern)
        FLmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveto(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;       // forward/back
        double x = gamepad.left_stick_x * 1.1;  // strafe
        double rx = -gamepad.right_stick_x;      // rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLpower = (y + x + rx) / denominator;
        double BLpower = (y - x + rx) / denominator;
        double FRpower = (y - x - rx) / denominator;
        double BRpower = (y + x - rx) / denominator;

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);

        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    }

    public void drivewithpower(double power) {
        FLmotor.setPower(1.0);
        FRmotor.setPower(1.0);
        BLmotor.setPower(1.0);
        BRmotor.setPower(1.0);
    }

}
