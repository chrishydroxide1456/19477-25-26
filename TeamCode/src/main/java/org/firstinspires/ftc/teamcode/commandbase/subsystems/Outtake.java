package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Outtake {

    private final DcMotorEx Tmotor, Bmotor;
//    private final Servo gateServo;
    private final RevBlinkinLedDriver led;

    public Outtake(HardwareMap hwMap) {
        Tmotor = hwMap.get(DcMotorEx.class, "Tmotor");
        Bmotor = hwMap.get(DcMotorEx.class, "Bmotor");
        Tmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Tmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Tmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Bmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Tmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.1, 0.1, 0.5));
//        Bmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.1, 0.1, 0.5));



        led = hwMap.get(RevBlinkinLedDriver.class, "led");
//        gateServo = hwMap.get(Servo.class, "gateServo");
//        gateServo.setPosition(0.0);

    }

    public void outtakeOn() {
//        Tmotor.setVelocity(3000.0 * 28.0 / 60.0);
//        Bmotor.setVelocity(3000.0 * 28.0 / 60.0);
          Tmotor.setPower(0.35);
          Bmotor.setPower(0.35);
    }

    public void outtakeOff() {
        Tmotor.setVelocity(0.0);
        Bmotor.setVelocity(0.0);
    }

//    public void closeGate() {
//        gateServo.setPosition(0.0);
//        }
//
//    public void openGate() {
//        gateServo.setPosition(1.0);
//        }

    public void maxRPM() {
        double TmotorVel = Tmotor.getVelocity();
        double BmotorVel = Bmotor.getVelocity();
        if (TmotorVel > 4800.0 * 28.0 / 60.0 && BmotorVel > 4800.0 * 28.0 / 60.0) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    }

