package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive implements Subsystem {

    public static final Drive INSTANCE = new Drive();
    public static double multi = 1.0;
    private Drive() {}

    private final MotorEx FLmotor = new MotorEx("FLmotor").brakeMode();
    private final MotorEx FRmotor = new MotorEx("FRmotor").reversed().brakeMode();
    private final MotorEx BLmotor = new MotorEx("BLmotor").reversed().brakeMode();
    private final MotorEx BRmotor = new MotorEx("BRmotor").reversed().brakeMode();

    private boolean autoAlignActive = false;

    public void setMulti(double newMulti) { multi = newMulti; }
    public void startAutoAlign() { autoAlignActive = true; }
    public void stopAutoAlign() { autoAlignActive = false; }
    public boolean isAutoAlignActive() { return autoAlignActive; }

    public void driverdrive(Gamepad gamepad) {
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

        double FLpower = (-y + x + rx) * multi;
        double BLpower = (y + x - rx) * multi;
        double FRpower = (-y - x - rx) * multi;
        double BRpower = (y - x + rx) * multi;

        if (!autoAlignActive) {
            FLmotor.setPower(FLpower);
            FRmotor.setPower(FRpower);
            BLmotor.setPower(BLpower);
            BRmotor.setPower(BRpower);
        } else {
            autoAlign(headingAdjust, FLpower, FRpower, BLpower, BRpower);
        }
    }

    // Very visible auto-align: robot WILL turn if headingAdjust != 0
    private void autoAlign(double headingErrorDeg,
                           double FLtrans, double FRtrans,
                           double BLtrans, double BRtrans) {

        // Start with large-ish P just to see motion; tune down later
        double kP = 0.05;
        double turnPower = kP * headingErrorDeg;

        if (turnPower > 0.6) turnPower = 0.6;
        if (turnPower < -0.6) turnPower = -0.6;

        // For now: 100% focus on turning when autoAlign is on
        double blend = 1.0;

        double FL = FLtrans * (1 - blend) + (-turnPower) * blend;
        double FR = FRtrans * (1 - blend) + ( turnPower) * blend;
        double BL = BLtrans * (1 - blend) + (-turnPower) * blend;
        double BR = BRtrans * (1 - blend) + ( turnPower) * blend;

        FLmotor.setPower(FL);
        FRmotor.setPower(FR);
        BLmotor.setPower(BL);
        BRmotor.setPower(BR);
    }

    // Optional helper
    public void autodrive(double rx) {
        double FLpower = -rx;
        double BLpower = rx;
        double FRpower = rx;
        double BRpower = -rx;

        FLmotor.setPower(FLpower);
        FRmotor.setPower(FRpower);
        BLmotor.setPower(BLpower);
        BRmotor.setPower(BRpower);
    }
}
