package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private Drive() { }
    
    // Motor directions - adjust .reversed() calls based on your robot's physical motor mounting
    // Current setup: FR and BR reversed (common for many mecanum configurations)
    // If robot doesn't drive correctly, try reversing different motors
    private final MotorEx frontLeftMotor = new MotorEx("FLmotor");
    private final MotorEx frontRightMotor = new MotorEx("FRmotor").reversed();
    private final MotorEx backLeftMotor = new MotorEx("BLmotor");
    private final MotorEx backRightMotor = new MotorEx("BRmotor").reversed();

    @Override
    public void initialize() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Constants for drive tuning
    private static final double STRAFE_CORRECTION = 1.1;  // Correction factor for mecanum strafe imperfection
    private static final double AUTO_ROTATION_DAMPING = 0.5;  // Reduce rotation sensitivity in autonomous

    /**
     * Mecanum drive controlled by gamepad
     * Left stick: translation (forward/back, strafe left/right)
     * Right stick X: rotation
     */
    public void driveWithGamepad(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;  // Forward/backward
        double x = gamepad.left_stick_x * STRAFE_CORRECTION;  // Strafe (with correction for mecanum imperfection)
        double rx = gamepad.right_stick_x;  // Rotation

        // Mecanum drive calculations
        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        // Normalize powers to keep within [-1, 1] range
        double maxPower = Math.max(Math.abs(frontLeftPower), 
                          Math.max(Math.abs(backLeftPower),
                          Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Manual drive control for autonomous (with rotation damping)
     */
    public void driveManual(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + (rotate * AUTO_ROTATION_DAMPING);
        double backLeftPower = forward - strafe + (rotate * AUTO_ROTATION_DAMPING);
        double frontRightPower = forward - strafe - (rotate * AUTO_ROTATION_DAMPING);
        double backRightPower = forward + strafe - (rotate * AUTO_ROTATION_DAMPING);

        // Normalize powers to keep within [-1, 1] range
        double maxPower = Math.max(Math.abs(frontLeftPower), 
                          Math.max(Math.abs(backLeftPower),
                          Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Stop all drive motors
     */
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

}
