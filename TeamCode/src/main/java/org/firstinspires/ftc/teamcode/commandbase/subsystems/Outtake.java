package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;

import com.qualcomm.robotcore.hardware.DcMotor;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}
    
    private final MotorEx topMotor = new MotorEx("Tmotor").reversed();
    private final MotorEx bottomMotor = new MotorEx("Bmotor");
    private final ServoEx gateServo = new ServoEx("gateServo");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.01, 0.0, 0.0) //need to tune
            .basicFF(0.01, 0.02, 0.03) //need to tune
            .build();

    @Override
    public void initialize() {
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateServo.setPosition(0.0);  // Closed position
    }

    // Commands as fields (following reference pattern)
    public final Command start = new RunToVelocity(controller, targetVel).requires(this);
    public final Command stop = new RunToVelocity(controller, 0.0).requires(this);
    public final Command open = new InstantCommand(() -> gateServo.setPosition(1.0)).requires(this);
    public final Command close = new InstantCommand(() -> gateServo.setPosition(0.0)).requires(this);

    @Override
    public void periodic() {
        topMotor.setPower(controller.calculate(topMotor.getState()));
        bottomMotor.setPower(controller.calculate(bottomMotor.getState()));
    }

}
