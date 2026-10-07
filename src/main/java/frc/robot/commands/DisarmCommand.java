package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CannonSubsystem;

/**
 * Disarm the cannon and dump tank pressure by opening vent for a short interval.
 * Use this when you want to make the cannon safe immediately.
 */
public class DisarmCommand extends Command {
    private final CannonSubsystem cannon;
    private final Timer timer = new Timer();

    private static final double DUMP_SECONDS = 10.0; // how long to open vent (tune for your tank)

    public DisarmCommand(CannonSubsystem cannon) {
        this.cannon = cannon;
        addRequirements(cannon);
    }

    @Override
    public void initialize() {
        // Disable closed-loop control right away
        cannon.setArmed(false);

        // Ensure charge is off and open vent to dump pressure
        cannon.chargeClose();
        cannon.ventOpen();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // nothing more — just wait for timer
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DUMP_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        // Close vent and ensure system left safe
        cannon.ventClose();
        cannon.chargeClose();
        timer.stop();
        timer.reset();
    }
}
