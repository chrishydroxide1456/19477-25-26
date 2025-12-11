package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.2)
            .forwardZeroPowerAcceleration(-27.8)
            .lateralZeroPowerAcceleration(-72.3)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.007, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.09, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0, 0.001, 0.7, 0.0))
//            .useSecondaryTranslationalPIDF(true) //staying on path
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true) //progress in path
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.58) //need to confirm
            .rightFrontMotorName("FRmotor")
            .rightRearMotorName("BRmotor")
            .leftRearMotorName("BLmotor")
            .leftFrontMotorName("FLmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //need to confirm directions later
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(50.6)
            .yVelocity(32.6);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //is this set?
            .forwardPodY(1.5)
            .strafePodX(-4.5)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}