package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CannonSubsystem;

/**
 * Fires the cannon for a short, safe pulse if conditions allow.
 */
public class FireCommand extends Command {
    private final CannonSubsystem cannon;
    private final Timer timer = new Timer();

    private static final double FIRE_TIME = 0.15; // duration in seconds for valve pulse

    public FireCommand(CannonSubsystem cannon) {
        this.cannon = cannon;
        addRequirements(cannon);
    }

    @Override
    public void initialize() {
        if (cannon.isArmed() && cannon.isPressureSafeToFire()) {
            System.out.println("🔥 Firing cannon!");
            cannon.fire();
            timer.restart();
        } else {
            System.out.println("❌ Cannot fire: not armed or insufficient pressure (" + cannon.getPressurePsi() + " PSI)");
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        cannon.ceaseFire();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(FIRE_TIME);
    }
}
