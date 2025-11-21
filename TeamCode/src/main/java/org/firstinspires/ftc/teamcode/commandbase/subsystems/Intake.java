package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}
    
    private final MotorEx motor = new MotorEx("inmotor");

    @Override
    public void initialize() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Commands as fields (following reference pattern)
    public final Command start = new InstantCommand(() -> motor.setPower(1.0)).requires(this);
    public final Command slowIn = new InstantCommand(() -> motor.setPower(0.4)).requires(this);
    public final Command slowOut = new InstantCommand(() -> motor.setPower(-0.2)).requires(this);
    public final Command stop = new InstantCommand(() -> motor.setPower(0.0)).requires(this);
    public final Command reverse = new InstantCommand(() -> motor.setPower(-0.8)).requires(this);

}
