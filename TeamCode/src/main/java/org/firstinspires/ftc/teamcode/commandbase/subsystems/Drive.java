package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    public static double multi = 1.0;
    private Drive() { }

    // EXACT SAME MOTOR SETUP AS YOUR ORIGINAL
    private final MotorEx FLmotor = new MotorEx("FLmotor");
    private final MotorEx FRmotor = new MotorEx("FRmotor").reversed();
    private final MotorEx BLmotor = new MotorEx("BLmotor").reversed();
    private final MotorEx BRmotor = new MotorEx("BRmotor").reversed();

    private boolean autoAlignActive = false;

    public void setMulti(double newMulti) {
        multi = newMulti;
    }

    public void startAutoAlign() { autoAlignActive = true; }
    public void stopAutoAlign() { autoAlignActive = false; }
    public boolean isAutoAlignActive() { return autoAlignActive; }

    public void driverdrive(Gamepad gamepad) {
        // AUTO ALIGN OVERRIDES EVERYTHING
        if (autoAlignActive) {
            autoAlignPure();
            return;
        }

        // YOUR EXACT ORIGINAL DRIVING CODE
        if (gamepad == null) {
            FLmotor.setPower(0);
            FRmotor.setPower(0);
            BLmotor.setPower(0);
            BRmotor.setPower(0);
            return;
        }

        double y = -gamepad.left_stick_y;
        double x = -gamepad.left_stick_x * 1.1;
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

    // Pure auto-align rotation (only runs when autoAlignActive = true)
    // Pure auto-align rotation (only runs when autoAlignActive = true)
    private void autoAlignPure() {
        double turnPower = 0.0;

        double error = headingAdjust;      // degrees from LL
        double kP = 0.02;                  // softer P gain (was 0.04)
        double maxPower = 0.5;             // lower max turn power (was 0.75)
        double minPower = 0.12;            // minimum to overcome friction

        if (Math.abs(error) > 0.7 && Math.abs(error) < 40.0) {
            turnPower = kP * error;

            // clamp to max magnitude
            if (turnPower > maxPower) turnPower = maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;

            // enforce small minimum power so it doesn’t stall
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }
        } else {
            // inside deadband → no turning
            turnPower = 0.0;
        }

        double FL = -turnPower;
        double BL =  turnPower;
        double FR =  turnPower;
        double BR = -turnPower;

        FLmotor.setPower(FL);
        FRmotor.setPower(FR);
        BLmotor.setPower(BL);
        BRmotor.setPower(BR);
    }


    public void autodrive(double rx) {
        // YOUR ORIGINAL autodrive
        double FLpower = (-rx);
        double BLpower = (rx);
        double FRpower = (rx);
        double BRpower = (-rx);

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);
        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    }
}
