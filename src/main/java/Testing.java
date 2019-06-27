import org.usfirst.frc.team3310.utility.control.Kinematics;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.math.Twist2d;

public class Testing {
	public static void main(final String... args) {
		final var state = RobotState.getInstance();

		state.addObservations(0.44, new Twist2d(0, 0, 0), Kinematics.forwardKinematics(0, 0));
		state.addObservations(1.22, new Twist2d(100, 0, 0), Kinematics.forwardKinematics(100, 100));
		state.addObservations(5.12, new Twist2d(500, 0, 0), Kinematics.forwardKinematics(125, 125));

		System.out.println(state.getLatestFieldToVehicle().toString());
	}
}
