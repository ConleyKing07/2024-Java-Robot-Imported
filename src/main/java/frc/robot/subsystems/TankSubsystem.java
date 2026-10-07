package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankSubsystem extends SubsystemBase {
    private final SparkMax leftFront;
    private final SparkMax leftBack;
    private final SparkMax rightFront;
    private final SparkMax rightBack;

    private final DifferentialDrive differentialDrive;

    // ===== NEW: Digital Output for Reverse Indicator =====
    private final DigitalOutput reverseDIO;

    public TankSubsystem() {
        reverseDIO = new DigitalOutput(1);

        // ========== Left Front ==========
        SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
        leftFrontConfig.inverted(true);
        leftFrontConfig.idleMode(IdleMode.kBrake);
        leftFrontConfig.smartCurrentLimit(40);
        leftFront = new SparkMax(2, MotorType.kBrushed);
        leftFront.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ========== Left Back ==========
        SparkMaxConfig leftBackConfig = new SparkMaxConfig();
        leftBackConfig.follow(2);
        leftBack = new SparkMax(3, MotorType.kBrushed);
        leftBack.configure(leftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ========== Right Front ==========
        SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
        rightFrontConfig.inverted(false);
        rightFrontConfig.smartCurrentLimit(40);
        rightFrontConfig.idleMode(IdleMode.kBrake);
        rightFront = new SparkMax(4, MotorType.kBrushed);
        rightFront.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ========== Right Back ==========
        SparkMaxConfig rightBackConfig = new SparkMaxConfig();
        rightBackConfig.follow(4);
        rightBack = new SparkMax(5, MotorType.kBrushed);
        rightBack.configure(rightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ========== DifferentialDrive ==========
        differentialDrive = new DifferentialDrive(leftFront, rightFront);
        differentialDrive.setDeadband(0.05);
    }

    /** Arcade drive (fwd/back + turning) */
    public void arcadeDrive(double forward, double rotation) {

        // ===== NEW: Set digital output based on forward direction =====
        if (forward < -0.2) {   // Reverse threshold
            reverseDIO.set(true);     // driving reverse
        } else {
            reverseDIO.set(false);    // not reverse
        }

        differentialDrive.arcadeDrive(forward, rotation);
    }

    /** Stop motors */
    public void stop() {
        differentialDrive.stopMotor();
        reverseDIO.set(false);   // ensure output is off when stopped
    }
}
