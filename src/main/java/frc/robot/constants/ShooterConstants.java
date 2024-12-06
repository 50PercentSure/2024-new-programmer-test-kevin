package frc.robot.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterConstants {

    public static final int topMotorID = 9;
    public static final int bottomMotorID = 10;

    public static final double kP = 0.001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double maxAcceleration = 1.0;
    public static final double maxVelocity = 1.0;

    TrapezoidProfile.Constraints shooterConstraints = new TrapezoidProfile.Constraints(maxAcceleration, maxVelocity);

    ProfiledPIDController shooterPIDController = new ProfiledPIDController(kP, kI, kD, shooterConstraints);

    // Motor to wheel ratio 60:36

}
