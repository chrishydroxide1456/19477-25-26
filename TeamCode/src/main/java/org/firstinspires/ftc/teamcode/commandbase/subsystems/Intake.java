package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private final DcMotorEx inmotor;

    public Intake(HardwareMap hwMap) {
        inmotor = hwMap.get(DcMotorEx.class, "inmotor");
        inmotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void dointake(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double inpower = y;

        inmotor.setPower(inpower);
    }


//    public void intakeOn() {
//        inmotor.setPower(-1.0);
//    }
//
//    public void intakeOff() {
//        inmotor.setPower(0.0);
//    }
//    public void intakeReverse() {
//        inmotor.setPower(1.0);
    }