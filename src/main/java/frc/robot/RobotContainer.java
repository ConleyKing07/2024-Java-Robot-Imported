package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    private final CommandPS5Controller scoringPS5 = new CommandPS5Controller(0);

    private final TankSubsystem drivetrain = new TankSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final CannonSubsystem cannon = new CannonSubsystem();

    private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, scoringPS5);
    private final ArmControlCommand armControlCommand = new ArmControlCommand(armSubsystem, scoringPS5);

    public RobotContainer() {
        ShuffleboardTab cannonTab = Shuffleboard.getTab("Cannon");

        // Add live widgets
        GenericEntry targetPressureEntry = cannonTab.add("Target PSI", cannon.getTargetPressure())
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", CannonSubsystem.MAX_SAFE_PSI))
                .getEntry();

        GenericEntry currentPressureEntry = cannonTab.add("Current PSI", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        GenericEntry armedStateEntry = cannonTab.add("Armed", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();

        // Link Shuffleboard entries with subsystem
        cannon.setShuffleboardEntries(targetPressureEntry, currentPressureEntry, armedStateEntry);

        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        scoringPS5.triangle().onTrue(new ArmCannonCommand(cannon));     // Arm toggle
        scoringPS5.circle().onTrue(new DisarmCommand(cannon));        // Disarm/dump
        scoringPS5.cross().onTrue(new FireEjectSequence(cannon));    // Fire
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(armControlCommand);
    }

    public CannonSubsystem getCannon() {
        return cannon;
    }
}
