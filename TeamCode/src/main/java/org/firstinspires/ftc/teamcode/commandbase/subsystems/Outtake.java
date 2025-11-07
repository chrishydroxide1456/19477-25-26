package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


@TeleOp
public class Outtake {

    private DcMotorEx Tmotor, Bmotor;
    private Servo gateServo;
    private boolean yeslaunch = false;
    private boolean yesgate = false;
    private boolean lastgateState = false;

    private RevBlinkinLedDriver led;

    public Outtake(HardwareMap hwMap) {
        Tmotor = hwMap.get(DcMotorEx.class, "Tmotor");
        Bmotor = hwMap.get(DcMotorEx.class, "Bmotor");
        gateServo = hwMap.get(Servo.class, "gateServo");
        Tmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        led = hwMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void launch(GamepadEx gamepad) {
        yeslaunch = gamepad.getButton(GamepadKeys.Button.A);
        if (yeslaunch) {
            Tmotor.setPower(1.0);
            Bmotor.setPower(1.0);
        }
        else {
            Tmotor.setPower(0.0);
            Bmotor.setPower(0.0);
        }
    }

    public void togglegate(GamepadEx gamepad) {
        boolean currentgateState = gamepad.getButton(GamepadKeys.Button.B);
        if (currentgateState && !lastgateState) {
            currentgateState = !currentgateState;
            gateServo.setPosition(0.3);
        }
        else {
            gateServo.setPosition(0.0);
        }
        lastgateState = currentgateState;
    }
    public boolean isyesgate() {
        return yesgate;
    }

    public void maxRPM() {
        double TmotorVel = Tmotor.getVelocity();
        double BmotorVel = Bmotor.getVelocity();
        if (TmotorVel > 4800*28/60 && BmotorVel > 4800*28/60) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

}