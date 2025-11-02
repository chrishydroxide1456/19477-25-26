package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class constants {

    public static double gateOpenDist = -1.0; //add in gateDist for gate servos later
    public static double gateCloseDist = 1.0;

    public static double kP = ;//add in velocity constants later
    public static double kI = ;
    public static double kD = ;
    public static double COUNTS_PER_REV_312 = 28;
    public static double targetVelocityTPS = (6000 / 60) * COUNTS_PER_REV_312;

    public static PIDFCoefficients FLYWHEEL_PIDF_CONSTANTS = new PIDFCoefficients(, , ,); //add in later
}
