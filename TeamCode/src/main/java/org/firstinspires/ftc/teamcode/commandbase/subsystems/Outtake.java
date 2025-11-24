package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}
    private final MotorEx Tmotor = new MotorEx("Tmotor").reversed();
    private final MotorEx Bmotor = new MotorEx("Bmotor");
    private final ServoEx gateServo = new ServoEx("gateServo");

    public void initialize() {
        gateServo.setPosition(0.0);
    }
    private final ControlSystem outcontroller = ControlSystem.builder()
            .velPid(0.01, 0.0, 0.0) //need to tune
            .basicFF(0.01, 0.02, 0.03) //need to tune
            .build();

    //public Command on = new RunToVelocity(outcontroller, 500);
    public Command on = new RunToVelocity(outcontroller, targetVel);

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
        }

        @Override
        public boolean isDone() {
            if (Bmotor.getVelocity() == 0) {
                return true;
            }
            else {
                return false;
            }
        }

        public void stop(boolean interrupted) {
            new RunToVelocity(outcontroller, 0.0); //is if interrupted = true necessary?
        }

    };

    public Command open = new SetPosition(gateServo, 1.0);
    public Command close = new SetPosition(gateServo, 0.0);
    public Command reverse = new RunToVelocity(outcontroller, -50).requires(this);


    public void periodic() {
        Tmotor.setPower(outcontroller.calculate(Tmotor.getState()));
        Bmotor.setPower(outcontroller.calculate(Bmotor.getState()));
    }

}
