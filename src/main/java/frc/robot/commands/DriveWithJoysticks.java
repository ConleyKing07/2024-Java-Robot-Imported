package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class DriveWithJoysticks extends Command {
    private final TankSubsystem drivetrain;
    private final CommandPS5Controller controller;

    public DriveWithJoysticks(TankSubsystem drivetrain, CommandPS5Controller scoringPS5) {
        this.drivetrain = drivetrain;
        this.controller = scoringPS5;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double forward = -controller.getLeftY();
        double rotation = -controller.getRightX();

    
        drivetrain.arcadeDrive(forward, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}

