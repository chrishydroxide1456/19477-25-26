package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.*;
import static dev.nextftc.bindings.Bindings.button;
import static org.firstinspires.ftc.teamcode.commandbase.Routines.overriding;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake.shooting;

import static java.lang.Math.sqrt;
//asdf lmao
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.*;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

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
public class BlueTeleOp extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive; //may remove later
    private Routines routines;


    public Follower follower;
    //public static Pose telestartingPose; //set later, see auto ex
    public static Pose telestartingPose = new Pose(27.0, 70.0, Math.toRadians(180.0));
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

    public static double shooterHeightIn     = 14.0;  // muzzle height
    public static double goalHeightIn        = 47.0;  // goal center height

    // Pass-through point relative to goal (inches)
    public static double passThroughRadiusIn = 8.0;  // back from goal along +Y
    public static double passThroughHeightIn = 50.0;  // height of pass-through

    // Hood angle limits (deg) and servo endpoints
    public static double minHoodDeg   = 37.288274;
    public static double maxHoodDeg   = 55.861336;
    public static double minHoodServo = 0.0;
    public static double maxHoodServo = 0.15;

    private static final double FIELD_X_GOAL_IN = 13.0;
    private static final double FIELD_Y_GOAL_IN = 138.0;

    // Robot velocity estimate in field frame (in/s)
    private Pose lastPose = null;
    private double lastPoseTime = 0.0;
    private double robotVx = 0.0;  // in/s, field X
    private double robotVy = 0.0;  // in/s, field Y

    private double estTOF = 0.4; //need to tune

    private double bufferDistIn = 50.0;
    
    // Distance to goal (horizontal, in inches) after smoothing
    public static double goalDistIn;

    // helper: hood angle → servo
    private double hoodDegToServo(double deg) {

        if (goalDistIn < bufferDistIn) {
            deg = maxHoodDeg;
        }

        double t = (deg - minHoodDeg) / (maxHoodDeg - minHoodDeg);
        t = Math.max(0.0, Math.min(1.0, t));
        return minHoodServo + t * (maxHoodServo - minHoodServo);
    }


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

                            .addPath(new Path (new BezierCurve(follower::getPose, new Pose(39.2, 76.32), new Pose (24.0, 65.0))))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180.0), 0.8))
                            .setVelocityConstraint(5.0)
                            .setBrakingStrength(1.0)
                            .setBrakingStart(0.75);
                    break;

                case gateintakePose:
                    builder
                            .addPath(new Path(new BezierLine(follower::getPose, new Pose(11.5, 60.5))))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(52), 0.6))
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

        // RIGHT BUMPER = OPEN SHOOT
        button(() -> gamepad2.right_bumper).whenBecomesTrue(() -> {
            routines.testoutSequence().schedule();
        });

        // Y = SPIN
        button(() -> gamepad2.y).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> shooting = true)
                .whenBecomesFalse(() -> shooting = false);

//        // DPAD UP = TOGGLE PRESPINUP
//        button(() -> gamepad2.dpad_up).toggleOnBecomesTrue()
//                .whenBecomesTrue(() -> prespinup = true)
//                .whenBecomesFalse(() -> prespinup = false);

        //DPAD UP = RELOCALIZE
        button(() -> gamepad2.dpad_up).whenBecomesTrue(() -> {
            follower.setPose(new Pose(8.0, 8.0, 90)); //with intake facing the wall
        });

        // A = INTAKE TOGGLE
        button(() -> gamepad2.a).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.on.schedule())
                .whenBecomesFalse(() -> {
                    intake.off2.schedule();
                    intake.keeping.schedule();
                });

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
        double now = System.nanoTime() / 1e9;

        if (lastPose != null) {
            double dt = now - lastPoseTime;
            if (dt > 0.01 && dt < 0.2) {
                robotVx = (pose.getX() - lastPose.getX()) / dt;
                robotVy = (pose.getY() - lastPose.getY()) / dt;
            }
        }
        lastPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        lastPoseTime = now;

        float rawDistance = 36.0f;
        double dxGoal = FIELD_X_GOAL_IN - pose.getX();
        double dyGoal = FIELD_Y_GOAL_IN - pose.getY();
        double distInches = Math.sqrt(dxGoal * dxGoal + dyGoal * dyGoal);

        if (!Double.isNaN(distInches) && !Double.isInfinite(distInches) && distInches > 0) {
            rawDistance = (float)Math.max(12.0, Math.min(120.0, distInches));
        }
        smoothedDistance = smoothDistance(rawDistance, smoothedDistance);
        distance = smoothedDistance;

        ShotSolution shot = computeShotInches(
                pose.getX(), pose.getY(),
                FIELD_X_GOAL_IN, FIELD_Y_GOAL_IN,
                robotVx, robotVy
        );

        double baseRPM = shot.rpm;
        double hoodServoPos = hoodDegToServo(shot.hoodDeg);
        outtake.setHoodPosition(hoodServoPos);


//        if (!Outtake.shooting) {
        if (distance > 105.0) {
            double calculatedVel = baseRPM;
            targetVel = (!Double.isNaN(calculatedVel) && !Double.isInfinite(calculatedVel))
                    ? 0.65 * calculatedVel + compensation : 800.0; // + 550 rpm when far
        } else {
            double calculatedVel = baseRPM;
            targetVel = (!Double.isNaN(calculatedVel) && !Double.isInfinite(calculatedVel))
                    ? 0.65 * calculatedVel + 250 : 800.0;
        }
//        }

        if (!automatedDrive) {
            if (autoaim) {
                double futureX = pose.getX() - robotVx * estTOF;
                double futureY = pose.getY() - robotVy * estTOF;

                double futuredX = 13.0 - futureX;
                double futuredY = 138.0 - futureY;

                targetHeading = Math.atan2(futuredY, futuredX) + Math.PI;
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
                        gamepad2.left_stick_y * multi,
                        -1.1 * gamepad2.left_stick_x * multi,
                        turnPower,
                        true

                );
            } else {
                follower.setTeleOpDrive(
                        gamepad2.left_stick_y * multi,
                        -1.1 * gamepad2.left_stick_x * multi,
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


    private static class ShotSolution {
        final double rpm;
        final double hoodDeg;

        ShotSolution(double rpm, double hoodDeg) {
            this.rpm = rpm;
            this.hoodDeg = hoodDeg;
        }
    }

    private ShotSolution computeShotInches(
            double robotX, double robotY,
            double goalX,  double goalY,
            double vRobotX, double vRobotY
    ) {
        // Use existing class fields directly:
        double shooterH = shooterHeightIn;
        double goalH    = goalHeightIn;

        double passX = goalX;
        double passY = goalY - passThroughRadiusIn;
        double passH = passThroughHeightIn;

        double dx = passX - robotX;
        double dy = passY - robotY;
        double horizontalDistIn = Math.hypot(dx, dy);
        double dz = passH - shooterH;

        double hoodAngleRad = Math.atan2(dz, horizontalDistIn);
        double hoodAngleDeg = Math.toDegrees(hoodAngleRad);

        hoodAngleDeg = Math.max(minHoodDeg, Math.min(maxHoodDeg, hoodAngleDeg));
        hoodAngleRad = Math.toRadians(hoodAngleDeg);

        double goalDx = goalX - robotX;
        double goalDy = goalY - robotY;
        goalDistIn = Math.hypot(goalDx, goalDy);

        double g = 9.8;
        double distM = goalDistIn / 39.37;
        double heightDiffM = (goalH - shooterH) / 39.37;
        double cosA = Math.cos(hoodAngleRad);
        double tanA = Math.tan(hoodAngleRad);
        double denom = distM * tanA - heightDiffM;
        if (denom <= 0.05) denom = 0.05;

        double numerator = g * distM * distM;
        double Vball_mps = Math.sqrt(numerator / (2.0 * cosA * cosA * denom));
        double Vball_ips = Vball_mps * 39.37;

        double ux = goalDx / goalDistIn;
        double uy = goalDy / goalDistIn;

        double vParallel      = vRobotX * ux + vRobotY * uy;
        double vPerpendicular = -vRobotX * uy + vRobotY * ux;

        double requiredHorizSpeed    = Vball_ips * Math.cos(hoodAngleRad);
        double compensatedHorizSpeed = requiredHorizSpeed + vParallel;

        double compensatedHorizMag = Math.hypot(compensatedHorizSpeed, vPerpendicular);
        double VballComp_ips = compensatedHorizMag / Math.cos(hoodAngleRad);
        double VballComp_mps = VballComp_ips / 39.37;

        double flywheelR_m = 2.0 / 39.37;   // 2 in radius in meters
        double Vwheel_mps  = VballComp_mps / efficiency;

        double rpm = (Vwheel_mps / (2.0 * Math.PI * flywheelR_m)) * 60.0;
        rpm = Math.max(800, Math.min(2800, rpm));

        return new ShotSolution(rpm, hoodAngleDeg);
    }

//    private double gettargetVel(double Distance) {
//        // Convert everything to meters
//        double Hshooter = 14.0 / 39.37;  // inches → meters
//        double Hgoal = 47.0 / 39.37;           // inches → meters
//
//        double launchangle = Math.toRadians(50);
//        double flywheelR = 2.0 / 39.37;                  // 2 inches → meters
//
//        double DistanceMeters = Distance / 39.37;
//        double heightDiff = Hgoal - Hshooter;
//
//        double cosAngle = Math.cos(launchangle);
//        double tanAngle = Math.tan(launchangle);
//        double denom = DistanceMeters * tanAngle - heightDiff;
//
//        // Safety check for invalid trajectories
//        if (denom <= 0.05) {
//            denom = 0.05;
//        }
//
//        // Ballistic calculation
//        double numerator = 9.8 * DistanceMeters * DistanceMeters;
//        double Vball = sqrt(numerator / (2.0 * cosAngle * cosAngle * denom));
//
//        // Account for energy loss (flywheel to ball efficiency)
//        double Vwheel = Vball / efficiency;
//
//        // Convert linear velocity (m/s) to RPM
//        double rpm = (Vwheel / (2.0 * Math.PI * flywheelR)) * 60.0;
//
//        // Clamp to motor limits
//        return Math.max(800, Math.min(2800, rpm));
//    }

    public float smoothDistance(float newDistance, float oldDistance) {
        return (float) (SMOOTHING_ALPHA * newDistance + (1.0 - SMOOTHING_ALPHA) * oldDistance);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

}