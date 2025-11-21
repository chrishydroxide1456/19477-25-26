package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * Simple outtake subsystem with 2 motors for shooting/scoring
 * and a servo blocker to control game element release
 */
public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private Outtake() {}
    
    // Two motors for outtake mechanism
    private final MotorEx topMotor = new MotorEx("Tmotor").reversed();
    private final MotorEx bottomMotor = new MotorEx("Bmotor");
    
    // Servo blocker to control element release
    private final ServoEx gateServo = new ServoEx("gateServo");

    @Override
    public void initialize() {
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateServo.setPosition(0.0);  // Start with blocker closed
    }

    // Motor control commands
    public final Command on = new InstantCommand(() -> {
        topMotor.setPower(1.0);
        bottomMotor.setPower(1.0);
    }).requires(this);
    
    public final Command off = new InstantCommand(() -> {
        topMotor.setPower(0.0);
        bottomMotor.setPower(0.0);
    }).requires(this);
    
    public final Command reverse = new InstantCommand(() -> {
        topMotor.setPower(-1.0);
        bottomMotor.setPower(-1.0);
    }).requires(this);
    
    // Servo blocker commands
    public final Command open = new InstantCommand(() -> gateServo.setPosition(1.0)).requires(this);
    public final Command close = new InstantCommand(() -> gateServo.setPosition(0.0)).requires(this);

}

