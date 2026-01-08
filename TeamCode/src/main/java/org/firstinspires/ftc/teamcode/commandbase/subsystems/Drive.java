package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

@Configurable
public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    public static double multi = 1.0;
    private Drive() { }

    public final MotorEx FLmotor = new MotorEx("FLmotor").floatMode();
    public final MotorEx FRmotor = new MotorEx("FRmotor").floatMode().reversed();
    public final MotorEx BLmotor = new MotorEx("BLmotor").floatMode().reversed();
    public final MotorEx BRmotor = new MotorEx("BRmotor").floatMode().reversed();

    public boolean autoAlignActive = false;

    // PID variables
    public double previousError = 0.0;
    public double integral = 0.0;
    public long lastTime = 0;

    // TUNED PID CONSTANTS (reduced overshoot)
    public static double kP = 0.042;              // Reduced from 0.055 (less aggressive = less overshoot)
    public static double kI = 0.0005;              // Small integral to eliminate steady-state error
    public static double kD = 0.000;              // INCREASED from 0.008 (more damping = less overshoot)

    public void setMulti(double newMulti) {
        multi = newMulti;
    }

    public void startAutoAlign() {
        autoAlignActive = true;
        // Reset PID state when starting
        previousError = 0.0;
        integral = 0.0;
        lastTime = System.currentTimeMillis();
    }

    public void stopAutoAlign() {
        autoAlignActive = false;
        // Reset PID state when stopping
        previousError = 0.0;
        integral = 0.0;
    }

    public boolean isAutoAlignActive() { return autoAlignActive; }

    public void driverdrive(Gamepad gamepad) {
        if (autoAlignActive) {
            autoAlignAggressive();
            return;
        }

        if (gamepad == null) {
            FLmotor.setPower(0);
            FRmotor.setPower(0);
            BLmotor.setPower(0);
            BRmotor.setPower(0);
            return;
        }

        double y = -gamepad.left_stick_y;
        double x = -gamepad.left_stick_x ;
        double rx = -gamepad.right_stick_x;
        double FLpower = (-y + x + rx);
        double BLpower = (y + x - rx);
        double FRpower = (-y - x - rx);
        double BRpower = (y - x + rx);

        FLmotor.setPower(FLpower * multi);
        FRmotor.setPower(FRpower * multi);
        BLmotor.setPower(BLpower * multi);
        BRmotor.setPower(BRpower * multi);
    }

    // IMPROVED: Reduced overshoot with better damping
    private void autoAlignAggressive() {
        double error = headingAdjust;  // degrees from LL

        double maxPower = 0.40;         // Reduced from 0.85 (prevent too much momentum)
        double minPower = 0.15;
        double deadband = 0.5;          // Slightly wider deadband (was 0.3)

        // Calculate time delta
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // seconds
        if (dt > 0.1 || dt <= 0) dt = 0.02; // Cap dt and handle first call
        lastTime = currentTime;

        double turnPower = 0.0;

        if (Math.abs(error) > deadband) {
            // PID calculation
            integral += error * dt;

            // Anti-windup: clamp integral
            double maxIntegral = 8.0;  // Reduced from 10.0
            if (integral > maxIntegral) integral = maxIntegral;
            if (integral < -maxIntegral) integral = -maxIntegral;

            double derivative = (error - previousError) / dt;

            // Full PID output
            turnPower = (kP * error) + (kI * integral) + (kD * derivative);

            // ADAPTIVE POWER LIMITING: reduce max power when close to target
            double adaptiveMaxPower = maxPower;
            if (Math.abs(error) < 3.0) {
                // Within 3 degrees: reduce max power to prevent overshoot
                adaptiveMaxPower = 0.2;
            } else if (Math.abs(error) < 7.0) {
                // Within 7 degrees: moderate power
                adaptiveMaxPower = 0.30;
            }

            // Clamp to adaptive max power
            if (turnPower > adaptiveMaxPower) turnPower = adaptiveMaxPower;
            if (turnPower < -adaptiveMaxPower) turnPower = -adaptiveMaxPower;

            // Apply minimum power threshold (only when not super close)
            if (Math.abs(error) > 2.0 && Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            previousError = error;
        } else {
            // Within deadband - stop and reset integral
            turnPower = 0.0;
            integral = 0.0;
            previousError = 0.0;
        }

        // Apply turn power to motors
        double FL = -turnPower;
        double BL =  turnPower;
        double FR =  turnPower;
        double BR = -turnPower;

        FLmotor.setPower(0.8*FL);
        FRmotor.setPower(0.8*FR);
        BLmotor.setPower(0.8*BL);
        BRmotor.setPower(0.8*BR);
    }

}