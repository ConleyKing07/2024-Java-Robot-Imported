package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final PIDController pidController;
    private final ArmFeedforward feedforward;

    private double targetPosition;

    // Max limits
    private final double MAX_POSITION = 225;
    private final double MIN_POSITION = 0;
    private final double MAX_SPEED = 500;

    // PID constants
    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;

    // Feedforward constants
    private final double kS = 0.0;
    private final double kG = 0.1;
    private final double kV = 0.0;
    private final double kA = 0.0;

    public ArmSubsystem() {
        // Motor configuration
        final SparkMaxConfig armConfig = new SparkMaxConfig();
        armMotor = new SparkMax(6, MotorType.kBrushless);
        armConfig.inverted(false);
        armConfig.smartCurrentLimit(10);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
        pidController = new PIDController(kP, kI, kD);
        feedforward = new ArmFeedforward(kS, kG, kV, kA);

        armMotor.getEncoder().setPosition(0);
    }

    public double getCurrentPosition() {
        return armMotor.getEncoder().getPosition();
    }

    public void setTargetPosition(double target) {
        this.targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, target));
    }

    public void update() {
        double currentPosition = armMotor.getEncoder().getPosition();
        currentPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, currentPosition));

        double pidOutput = pidController.calculate(currentPosition, targetPosition);
        pidOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, pidOutput));

        // Estimate velocity and acceleration if needed
        double velocity = 0;     // You can estimate this if you track previous positions
        double acceleration = 0; // Same as above

        // Calculate feedforward in volts
        double ffVolts = feedforward.calculate(currentPosition, velocity, acceleration);

        // Combine PID + feedforward, convert voltage to percent output if needed
        double totalOutput = pidOutput + ffVolts / 12.0;
        totalOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, totalOutput));

        armMotor.set(totalOutput);

        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("PID Output", pidController.getSetpoint());
    }

    @Override
    public void periodic() {
        update();
    }

    public void log() {
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("PID Output", pidController.getSetpoint());
    }
}
