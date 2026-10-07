package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class ArmControlCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final CommandPS5Controller controller;

    // Constants for joystick deadzone and speed scaling
    private static final double DEADZONE = 0.1;
    private static final double SPEED_SCALE = 10;

    public ArmControlCommand(ArmSubsystem armSubsystem, CommandPS5Controller scoringPS5) {
        this.armSubsystem = armSubsystem;
        this.controller = scoringPS5;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double joystickY = -controller.getRightY(); // Invert for natural up/down
        if (Math.abs(joystickY) < DEADZONE) {
            joystickY = 0;
        }

        double speed = joystickY * SPEED_SCALE;
        armSubsystem.setTargetPosition(armSubsystem.getCurrentPosition() + speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setTargetPosition(armSubsystem.getCurrentPosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
