package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

    public ShooterSubsystem shooterSubsystem;
    public ProfiledPIDController shooterPIDController;
    public XboxController xboxController;
    public double wheelRPM;

    public double motorRPM = wheelRPM * ((double) 36 / 60);



    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.setTopMotorVolts(
                shooterPIDController.calculate(shooterSubsystem.getTopMotorRPM(), motorRPM)
        );
        shooterSubsystem.setBottomMotorVolts(
                shooterPIDController.calculate(shooterSubsystem.getBottomMotorRPM(), motorRPM)
        );
    }

    public void end() {
        shooterSubsystem.setTopMotorVolts(0);
        shooterSubsystem.setBottomMotorVolts(0);
    }

}
