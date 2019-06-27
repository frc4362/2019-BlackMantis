package org.usfirst.frc.team3310.utility.control;

import com.gemsrobotics.Hardware;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DifferentialDrive.Side;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Twist2d;

import java.util.Objects;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
// I made some changes to this so that we can use it with our DriveTrain
public class RobotStateEstimator extends Command {
    private static RobotStateEstimator instance_;

    public static RobotStateEstimator getInstance() {
        if (Objects.isNull(instance_)) {
             instance_ = new RobotStateEstimator();
        }

        return instance_;
    }

    private final RobotState robot_state_ = RobotState.getInstance();
    private final DifferentialDrive drive_ = Hardware.getInstance().getChassis();

    private double left_encoder_prev_distance_;
    private double right_encoder_prev_distance_;

    @Override
    public void initialize() {
        left_encoder_prev_distance_ = drive_.getInchesPosition(Side.LEFT);
        right_encoder_prev_distance_ = drive_.getInchesPosition(Side.RIGHT);
    }

    @Override
    public void execute() {
        final double left_distance = drive_.getInchesPosition(Side.LEFT);
        final double right_distance = drive_.getInchesPosition(Side.RIGHT);

        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_,
                right_distance - right_encoder_prev_distance_,
                drive_.getRotation());

        final Twist2d predicted_velocity = Kinematics.forwardKinematics(
                drive_.getInchesPerSecond(Side.LEFT),
                drive_.getInchesPerSecond(Side.RIGHT));

        robot_state_.addObservations(Timer.getFPGATimestamp(), odometry_velocity, predicted_velocity);

        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
