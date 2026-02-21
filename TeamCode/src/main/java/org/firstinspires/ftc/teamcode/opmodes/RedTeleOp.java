package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.*;
import static dev.nextftc.bindings.Bindings.button;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake.shooting;

import static java.lang.Math.sqrt;
//asdf lmao
import dev.nextftc.core.components.*;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.Supplier;
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class RedTeleOp extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive; //may remove later
    private Routines routines;


    public Follower follower;
    //public static Pose telestartingPose; //set later, see auto ex
    public static Pose telestartingPose = new Pose(104.0, 112.0, Math.toRadians(0.0));
    private boolean automatedDrive = false;

    private boolean autoaim = false;

    private static double targetHeading;
    private static double headingError;
    private static double rawError;
    private Supplier<PathChain> pathChain;
    private double multi = 0.8; //tune later
    public static double turnPower;

    private static final double SMOOTHING_ALPHA = 0.5;
    private float smoothedDistance;

    private enum endPose {
        gatePose, gateintakePose, parkPose
    }

    private endPose endpose;

    public static boolean prespinup = false;

    private static double kP = 0.75;
    private static double kD = 0.05;

    private double lasttime = 0.0;
    private double preverror = 0.0;

    @Override
    public void onInit() {
        LL.ID = 24;
        targetVel = 0.0;
        smoothedDistance = 36.0f;

        Drive.INSTANCE.stopAutoAlign();

        intake = Intake.INSTANCE;
        outtake = Outtake.INSTANCE;
        outtake.initialize(hardwareMap);
        ll = LL.INSTANCE;
        ll.initialize(hardwareMap);
        drive = Drive.INSTANCE;
        routines = new Routines(intake, outtake, ll, drive);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(telestartingPose == null ? new Pose() : telestartingPose);
        follower.update();


        pathChain = () -> {
            PathBuilder builder = follower.pathBuilder();

            switch (endpose) {
                case gatePose:
                    builder
                            //.addPath(new Path(new BezierLine(follower::getPose, new Pose(128.5, 66))))
                            .addPath(new Path (new BezierCurve(follower::getPose, new Pose(101.84, 76.32), new Pose (120.0, 65.0))))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0.0), 0.8)) //finish rotating at 80% thru path
                            .setVelocityConstraint(5.0)
                            .setBrakingStrength(1.0)
                            .setBrakingStart(0.75);
                    break;

                case gateintakePose:
                    builder
                            .addPath(new Path(new BezierLine(follower::getPose, new Pose(133, 60.5))))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(52-180), 0.6))
                            .setVelocityConstraint(5.0)
                            .setBrakingStrength(1.0)
                            .setBrakingStart(0.75);
                    break;

            }

            return builder.build();
        };



        addComponents(
                new SubsystemComponent(ll, drive, intake, outtake),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
                //new PedroComponent(Constants::createFollower)
        );

        //LEFT BUMPER = AUTOALIGN
//        button(() -> gamepad2.left_bumper)
//                .whenTrue(() -> {
//                    if (LL.tagVisible && Math.abs(headingAdjust) > 0.5) {  // Use LL.tagVisible directly
//                        drive.autoAlignActive = true;
//                        drive.previousError = 0.0;
//                        drive.integral = 0.0;
//                        drive.lastTime = System.currentTimeMillis();
//                    }
//                })
//                .whenFalse(() -> {
//                    drive.autoAlignActive = false;
//                });

        button(() -> gamepad2.left_bumper).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> autoaim = true)
                .whenBecomesFalse(() -> autoaim = false);

        // RIGHT BUMPER = SHOOT SEQUENCE
        button(() -> gamepad2.right_bumper).whenBecomesTrue(() -> {
                routines.testoutSequence().schedule();
        });

        // Y = PRESPIN
        button(() -> gamepad2.y).toggleOnBecomesTrue()
            .whenBecomesTrue(() -> shooting = true)
            .whenBecomesFalse(() -> shooting = false);

//        // DPAD UP = TOGGLE PRESPINUP
//        button(() -> gamepad2.dpad_up).toggleOnBecomesTrue()
//                .whenBecomesTrue(() -> prespinup = true)
//                .whenBecomesFalse(() -> prespinup = false);

        //DPAD UP = RELOCALIZE
        button(() -> gamepad2.dpad_up).whenBecomesTrue(() -> {
            follower.setPose(new Pose(136.0, 8.0, 180)); //with intake facing the wall
        });

        // A = INTAKE TOGGLE
        button(() -> gamepad2.a).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());

        // X = REVERSE INTAKE
        button(() -> gamepad2.x).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.reverse.schedule())
                .whenBecomesFalse(() -> routines.stopReverseSequence().schedule());

    }

    @Override
    public void onStartButtonPressed() {
        follower.startTeleOpDrive(false); //true or false? check Constants
    }

    @Override
    public void onUpdate() {
        follower.update();
        Pose pose = follower.getPose();

        float rawDistance = 36.0f;
        double distInches = sqrt(Math.pow((138.0-pose.getY()), 2.0) + Math.pow((132.0-pose.getX()), 2.0));
        if (!Double.isNaN(distInches) && !Double.isInfinite(distInches) && distInches > 0) {
            rawDistance = (float) Math.max(12.0, Math.min(120.0, distInches));
        }

        smoothedDistance = smoothDistance(rawDistance, smoothedDistance);
        distance = smoothedDistance;

//        if (!Outtake.shooting) {
        if (distance > 105.0) {
            double calculatedVel = gettargetVel(distance);
            targetVel = (!Double.isNaN(calculatedVel) && !Double.isInfinite(calculatedVel))
                    ? 0.65 * calculatedVel + compensation : 800.0; // + 550 rpm when far
        } else {
            double calculatedVel = gettargetVel(distance);
            targetVel = (!Double.isNaN(calculatedVel) && !Double.isInfinite(calculatedVel))
                    ? 0.65 * calculatedVel + 250 : 800.0;
        }
//        }

        if (!automatedDrive) {
            if (autoaim) {
                targetHeading = Math.atan2(138.0 - pose.getY(), 132.0 - pose.getX()) + Math.PI;
//                rawError = angleWrap();
                headingError = angleWrap(targetHeading - pose.getHeading());
                //drive.autoAlignAggressive(headingError);
                double dt = System.currentTimeMillis() / 1000.0 - lasttime;
                if (dt > 0.01) {
                    double derivative = (headingError - preverror) / dt;
                    turnPower = kP * headingError + kD * derivative;
                    turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
                    preverror = headingError;
                    lasttime = System.currentTimeMillis() / 1000.0;
                }

                follower.setTeleOpDrive(
                        -gamepad2.left_stick_y * multi,
                        -gamepad2.left_stick_x * multi,
                        turnPower,
                        true
                        
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad2.left_stick_y * multi,
                        -gamepad2.left_stick_x * multi,
                        -gamepad2.right_stick_x * multi,
                        true
                );
            }
        }



        //DPAD RIGHT = GO TO GATE
        if (gamepad2.dpadRightWasPressed()) {
            endpose = endPose.gatePose;
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //DPAD LEFT = GO TO GATE INTAKE POSITION
        if (gamepad2.dpadLeftWasPressed()) {
            endpose = endPose.gateintakePose;
            follower.followPath(pathChain.get()); //make path to intake position
            routines.inSequence().schedule();
            automatedDrive = true;
        }

        //DPAD DOWN = CANCEL
        if (automatedDrive && (gamepad2.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleOpDrive(false);
            automatedDrive = false;
        }


        // Telemetry
        double TmotorRPM = outtake.Tmotor.getVelocity();
        double BmotorRPM = outtake.Bmotor.getVelocity();
        double currentheading = follower.getHeading();
        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();


        telemetry.addLine("=== SHOOT SYSTEM ===");
        telemetry.addData("Distance", "%.1f in", distance);
        telemetry.addData("Calculated RPM", "%.0f", targetVel);
        telemetry.addData("local targetvel", "%.0f", Outtake.localTargetVel);
        telemetry.addData("Shooting", Outtake.shooting);
        telemetry.addData("Top Motor RPM", "%.0f", TmotorRPM);
        telemetry.addData("Top Motor Power", "%.3f", outtake.Tmotor.getPower());
        telemetry.addData("Top Error", "%.0f RPM", targetVel - TmotorRPM);
        telemetry.addData("Bottom Motor RPM", "%.0f", BmotorRPM);
        telemetry.addData("Bottom Motor Power", "%.3f", outtake.Bmotor.getPower());
        telemetry.addData("Bottom Error", "%.0f RPM", targetVel - BmotorRPM);

        telemetry.addLine("=== AUTOAIM ===");
        telemetry.addData("heading", "%.2f", currentheading);
        telemetry.addData("Error°", "%.1f", Math.toDegrees(headingError));
        telemetry.addData("turnPower", "%.2f", turnPower);
        telemetry.addData("autoaim", autoaim);
        telemetry.addData("currentX", currentX);
        telemetry.addData("currentY", currentY);

        telemetry.update();

        // REMOVED: The duplicate override logic that was causing the race condition
        // The LL subsystem now handles all targetVel setting in its periodic() method
    }

//    private double angleWrap(double radians) {
//        return ((radians + Math.PI) % (2.0 * Math.PI)) - Math.PI; //check this later (-pi + 180 degrees)
//    }

    private double gettargetVel(double Distance) {
        // Convert everything to meters
        double Hshooter = 14.0 / 39.37;  // inches → meters
        double Hgoal = 47.0 / 39.37;           // inches → meters
        double launchangle = Math.toRadians(50);
        double flywheelR = 2.0 / 39.37;                  // 2 inches → meters

        double DistanceMeters = Distance / 39.37;
        double heightDiff = Hgoal - Hshooter;

        double cosAngle = Math.cos(launchangle);
        double tanAngle = Math.tan(launchangle);
        double denom = DistanceMeters * tanAngle - heightDiff;

        // Safety check for invalid trajectories
        if (denom <= 0.05) {
            denom = 0.05;
        }

        // Ballistic calculation
        double numerator = 9.8 * DistanceMeters * DistanceMeters;
        double Vball = sqrt(numerator / (2.0 * cosAngle * cosAngle * denom));

        // Account for energy loss (flywheel to ball efficiency)
        double Vwheel = Vball / efficiency;

        // Convert linear velocity (m/s) to RPM
        double rpm = (Vwheel / (2.0 * Math.PI * flywheelR)) * 60.0;

        // Clamp to motor limits
        return Math.max(800, Math.min(2800, rpm));
    }

    public float smoothDistance(float newDistance, float oldDistance) {
        return (float) (SMOOTHING_ALPHA * newDistance + (1.0 - SMOOTHING_ALPHA) * oldDistance);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

}