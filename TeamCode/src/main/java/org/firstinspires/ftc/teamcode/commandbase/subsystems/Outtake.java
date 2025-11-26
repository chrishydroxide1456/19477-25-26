package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import androidx.annotation.NonNull;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}
    private final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    private final MotorEx Bmotor = new MotorEx("Bmotor");
    private final ServoEx gateServo = new ServoEx("gateServo");
    private final CRServoEx spinServo1 = new CRServoEx("spinServo1");
    private final CRServoEx spinServo2 = new CRServoEx("spinServo2");
    private RevBlinkinLedDriver led;


    public void initialize() {
        gateServo.setPosition(0.0);
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }
    private final ControlSystem outcontroller = ControlSystem.builder()
            .velPid(0.01, 0.0, 0.0) //need to tune
            .basicFF(0.01, 0.02, 0.03) //need to tune
            .build();

    //public Command on = new RunToVelocity(outcontroller, 500);
    //public Command on = new RunToVelocity(outcontroller, targetVel);
    public Command on = new Command() {

        @Override
        public void update() {
            new RunToVelocity(outcontroller, targetVel);
            new SetPower(spinServo1, 0.1); //need to tune later
            new SetPower(spinServo2, 0.1);

            if (Tmotor.getVelocity() > (targetVel - 50)) { //100 rpm = 46.67 tps
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);;
            }
            else {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }

        @Override
        public boolean isDone() {
            return false; //i think this is okay
        }

    };

    //public Command off = new RunToVelocity(outcontroller, 0.0);
    public Command off = new Command() {
        long t1 = System.currentTimeMillis();

        @Override
        public void update() {
            long t2 = System.currentTimeMillis();
            double dt = (double) (t2 - t1) /100;
            t1 = t2;

            if (Tmotor.getVelocity() > 0) {
                targetVel = Tmotor.getVelocity() - 400 * dt; //400 = slow-down rate
            }

            new RunToVelocity(outcontroller, targetVel);
            new SetPower(spinServo1, 0.0); //slew the CRServos too?
            new SetPower(spinServo2, 0.0);
        }

        @Override
        public boolean isDone() {
            return Bmotor.getVelocity() == 0;
        }

        public void stop(boolean interrupted) {
            new RunToVelocity(outcontroller, 0.0); //just in case
            new SetPower(spinServo1, 0.0);
            new SetPower(spinServo2, 0.0);
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    };

    public Command open = new SetPosition(gateServo, 0.7);
    public Command close = new SetPosition(gateServo, 0.0);
    public Command reverse = new RunToVelocity(outcontroller, -56).requires(this);


    public void periodic() {
        Tmotor.setPower(outcontroller.calculate(Tmotor.getState()));
        Bmotor.setPower(outcontroller.calculate(Bmotor.getState()));
    }

}
