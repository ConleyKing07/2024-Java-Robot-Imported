package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.CannonSubsystem;

/**
 * Fire -> open barrel -> short eject burst -> close barrel sequence.
 * Safety: only runs if cannon is armed and pressure safe to fire.
 */
public class FireEjectSequence extends SequentialCommandGroup {

    private static final double FIRE_PULSE_TIME = 0.2;   // seconds to pulse fire solenoid
    private static final double BARREL_OPEN_WAIT = 1.0;  // seconds to wait after opening barrel
    private static final double EJECT_BURST_TIME = 0.05;
    private static final double Close_Wait_Time = 1.25;  // short burst for ejector (adjustable)
    // note: no long wait after eject; we proceed to stopEject() then close barrel

    public FireEjectSequence(CannonSubsystem cannon) {
        // Core sequence
        SequentialCommandGroup coreSequence = new SequentialCommandGroup(
            // 1) Fire (energize)
            new InstantCommand(() -> cannon.fire(), cannon),
            // 2) Hold fire for a short pulse
            new WaitCommand(FIRE_PULSE_TIME),
            // 3) Cease fire
            new InstantCommand(() -> cannon.ceaseFire(), cannon),

            // 4) Open barrel
            new InstantCommand(() -> cannon.barrelOpen(), cannon),
            // 5) Wait for barrel to open
            new WaitCommand(BARREL_OPEN_WAIT),

            // 6) Eject shell with a short burst
            new InstantCommand(() -> cannon.eject(), cannon),
            new WaitCommand(EJECT_BURST_TIME),
            new InstantCommand(() -> cannon.stopEject(), cannon),
            new WaitCommand(Close_Wait_Time),

            // 7) Close barrel
            new InstantCommand(() -> cannon.barrelClose(), cannon)
        );

        // Only run sequence if safe
        ConditionalCommand safeSequence = new ConditionalCommand(
            coreSequence,
            new InstantCommand(() -> System.out.println("❌ FireEjectSequence aborted: not armed or insufficient pressure")),
            () -> cannon.isArmed() && cannon.isPressureSafeToFire()
        );

        addCommands(safeSequence);
    }
}
