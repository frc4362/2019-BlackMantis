package org.usfirst.frc.team3310.utility.control;

import com.gemsrobotics.Hardware;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
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
        left_encoder_prev_distance_ = drive_.getInchesPosition(DifferentialDrive.Side.LEFT);
        right_encoder_prev_distance_ = drive_.getInchesPosition(DifferentialDrive.Side.RIGHT);
    }

    @Override
    public void execute() {
        final double left_distance = drive_.getInchesPosition(DifferentialDrive.Side.LEFT);
        final double right_distance = drive_.getInchesPerSecond(DifferentialDrive.Side.RIGHT);
        final Rotation2d gyro_angle = Rotation2d.fromDegrees(-drive_.getAHRS().getAngle());
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_,
                right_distance - right_encoder_prev_distance_, gyro_angle
        );

        final Twist2d predicted_velocity = Kinematics.forwardKinematics(
                drive_.getInchesPerSecond(DifferentialDrive.Side.LEFT),
                drive_.getInchesPerSecond(DifferentialDrive.Side.RIGHT)
        );

        robot_state_.addObservations(Timer.getFPGATimestamp(), odometry_velocity, predicted_velocity);

        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
