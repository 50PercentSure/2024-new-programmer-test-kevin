package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

    public ShooterSubsystem shooterSubsystem;
    public ProfiledPIDController shooterPIDController;


    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {

    }

    public void end() {

    }

}
