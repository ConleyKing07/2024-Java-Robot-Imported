package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CannonSubsystem;

/**
 * Toggleable arm/disarm command for the cannon.
 * When executed, it toggles the subsystem's armed state once and finishes immediately.
 */
public class ArmCannonCommand extends Command {
    private final CannonSubsystem cannon;

    public ArmCannonCommand(CannonSubsystem cannon) {
        this.cannon = cannon;
        addRequirements(cannon);
    }

    @Override
    public void initialize() {
        boolean currentlyArmed = cannon.isArmed();
        cannon.setArmed(!currentlyArmed); // Toggle the state

        System.out.println("🔄 Cannon " + (cannon.isArmed() ? "ARMED" : "DISARMED") + " via toggle command");
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }
}
