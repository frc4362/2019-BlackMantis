package com.gemsrobotics;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
// originally from FRC3310
public class Constants {
    public static double kLooperDt = 0.02;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static double kDriveWheelDiameterInches = 5.8;
    public static double kTrackWidthInches = 27.75;
    public static double kTrackScrubFactor = 0.924;
    
    // Drive constants
    public static double kDriveLowGearMaxSpeedInchesPerSec = 12.0 * 10.0;

    // Geometry
    public static double kCenterToFrontBumperDistance = 19.25;
    public static double kCenterToRearBumperDistance = 19.25;
    public static double kCenterToSideBumperDistance = 17.125;

    /* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 0.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 0.04;
    public static double kDriveHighGearVelocityKf = 0.07;
    public static int kDriveHighGearVelocityIZone = 200;
    public static double kDriveHighGearVelocityRampRate = 0.05;
    public static double kDriveHighGearNominalOutput = 0.5/12.0;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0;

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearVelocityKp = 0.1;
    public static double kDriveLowGearVelocityKi = 0.0;
    public static double kDriveLowGearVelocityKd = 0.04;
    public static double kDriveLowGearVelocityKf = 0.03;
    public static int kDriveLowGearVelocityIZone = 200;
    public static double kDriveLowGearVelocityRampRate = 0.02;
    public static double kDriveLowGearNominalOutput = 0.1/12.0;
    public static double kDriveLowGearMaxSetpoint = 10.0 * 12.0;

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 1.0;
    public static double kDriveLowGearPositionKi = 0.002;
    public static double kDriveLowGearPositionKd = 0.04;
    public static double kDriveLowGearPositionKf = .45;
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 0.05; // V/s
    public static double kDriveLowGearMaxVelocity = 8.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 8 fps
                                                                                                               // in RPM
    public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s
    public static double kDriveVoltageCompensationRampRate = 0.0;

    // Turn to heading gains
    public static double kDriveTurnKp = 3.0;
    public static double kDriveTurnKi = 1.5;
    public static double kDriveTurnKv = 0.0;
    public static double kDriveTurnKffv = 1.0;
    public static double kDriveTurnKffa = 0.0;
    public static double kDriveTurnMaxVel = 360.0;
    public static double kDriveTurnMaxAcc = 720.0;

    // Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec

    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 90.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 5.0;  //5.0
    public static double kPathFollowingProfileKi = 0.03;  // 0.03
    public static double kPathFollowingProfileKv = 0.0083; //0.02
    public static double kPathFollowingProfileKffv = 1.2;  //1.2
    public static double kPathFollowingProfileKffa = 0.002;  //0.05
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;
}
