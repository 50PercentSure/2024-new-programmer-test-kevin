package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.topMotorID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.bottomMotorID, CANSparkLowLevel.MotorType.kBrushless);

    public ShooterSubsystem() {

        this.topMotor.setInverted(true);
        this.bottomMotor.setInverted(false);

        topMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

    }

    public void setVolts(double volts) {
        topMotor.setVoltage(volts);
        bottomMotor.setVoltage(volts);
    }

    public void setTopMotorRPM(double topMotorRPM) { topMotor.set(topMotorRPM); }

    public void setBottomMotorRPM(double bottomMotorRPM) { bottomMotor.set(bottomMotorRPM); }

    public double getTopMotorRPM() { return topMotor.getEncoder().getVelocity(); }
    public double getBottomMotorRPM() { return bottomMotor.getEncoder().getVelocity(); }
}
