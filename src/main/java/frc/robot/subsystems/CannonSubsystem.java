package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CannonSubsystem extends SubsystemBase {
    private static final int PH_CAN_ID = 1;
    private static final int BARREL_FORWARD = 4;
    private static final int BARREL_REVERSE = 5;
    private static final int FIRE_PORT = 1;
    private static final int CHARGE_PORT = 0;
    private static final int VENT_PORT = 2;
    private static final int EJECT_PORT = 3;
    private static final int PRESSURE_ANALOG_CHANNEL = 0;
    private static final double PRESSURE_SENSOR_MAX_PSI = 160;
    private static final int SIREN_CAN_ID = 7;
    public static final double MAX_SAFE_PSI = 160.0;
    private static final double MIN_FIRE_PSI = 35.0;
    private static final double TOLERANCE_PSI = 1.0;

    private final PneumaticHub ph;
    private final DoubleSolenoid barrel;
    private final Solenoid fireValve;
    private final Solenoid chargeValve;
    private final Solenoid ventValve;
    private final Solenoid ejectValve;

    private final PIDController pressurePid = new PIDController(0.1, 0.02, 0.4);
    private double targetPressure = 60;
    private boolean armed = false;

    // Siren
    private final SparkMax sirenMotor;
    private final Timer sirenTimer = new Timer();
    private static final double SIREN_MAX_SPEED = 1.0;
    private static final double SIREN_RAMP_RATE = 0.004;
    private static final double SIREN_BURST_TIME = 7;
    private double sirenTarget = 0.0;
    private double sirenCurrent = 0.0;

    // Strobe
    private final DigitalOutput strobeLight;

    // Shuffleboard-linked entries
    private GenericEntry targetPressureEntry;
    private GenericEntry currentPressureEntry;
    private GenericEntry armedStateEntry;

    @SuppressWarnings("deprecation")
    public CannonSubsystem() {
        ph = new PneumaticHub(PH_CAN_ID);

        barrel = ph.makeDoubleSolenoid(BARREL_FORWARD, BARREL_REVERSE);
        fireValve = ph.makeSolenoid(FIRE_PORT);
        chargeValve = ph.makeSolenoid(CHARGE_PORT);
        ventValve = ph.makeSolenoid(VENT_PORT);
        ejectValve = ph.makeSolenoid(EJECT_PORT);

        pressurePid.setTolerance(TOLERANCE_PSI);

        sirenMotor = new SparkMax(SIREN_CAN_ID, MotorType.kBrushless);
        sirenMotor.setInverted(false);
        sirenMotor.set(0);
        sirenTimer.start();

        strobeLight = new DigitalOutput(0);
        strobeLight.set(true);

        SmartDashboard.putNumber("Cannon/Target PSI", targetPressure);
        SmartDashboard.putBoolean("Cannon/Armed", armed);
    }

    // -------------------- Shuffleboard Link --------------------
    public void setShuffleboardEntries(GenericEntry targetPressureEntry, GenericEntry currentPressureEntry, GenericEntry armedStateEntry) {
        this.targetPressureEntry = targetPressureEntry;
        this.currentPressureEntry = currentPressureEntry;
        this.armedStateEntry = armedStateEntry;
    }

    // -------------- Basic Pneumatic Controls --------------
    public void barrelOpen()  { barrel.set(Value.kForward); }
    public void barrelClose() { barrel.set(Value.kReverse); }

    public void fire()      { fireValve.set(true); }
    public void ceaseFire() { fireValve.set(false); }

    public void chargeOpen()  { chargeValve.set(true); }
    public void chargeClose() { chargeValve.set(false); }

    public void ventOpen()  { ventValve.set(true); }
    public void ventClose() { ventValve.set(false); }

    public void eject()     { ejectValve.set(true); }
    public void stopEject() { ejectValve.set(false); }

    // -------------------- Pressure --------------------
    public double getPressurePsi() {
        double volts = ph.getAnalogVoltage(PRESSURE_ANALOG_CHANNEL);
        double psi = (volts / 5.0) * PRESSURE_SENSOR_MAX_PSI;
        return Math.max(0, Math.min(psi, MAX_SAFE_PSI));
    }

    public boolean isPressureSafeToFire() {
        return getPressurePsi() >= MIN_FIRE_PSI;
    }

    public void setArmed(boolean arm) {
        this.armed = arm;
        SmartDashboard.putBoolean("Cannon/Armed", armed);
        if (!arm) {
            chargeClose();
            ventClose();
        }
    }

    public boolean isArmed() { return armed; }

    public void setTargetPressure(double psi) {
        targetPressure = Math.max(0.0, Math.min(psi, MAX_SAFE_PSI));
        SmartDashboard.putNumber("Cannon/Target PSI", targetPressure);
    }

    public double getTargetPressure() { return targetPressure; }

    // -------------------- PID Control --------------------
    private void controlPressureLoop() {
        // Pull live target PSI from Shuffleboard (if linked)
        double liveTarget = (targetPressureEntry != null)
            ? targetPressureEntry.getDouble(targetPressure)
            : NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Cannon/Target PSI").getDouble(targetPressure);

        targetPressure = Math.max(0.0, Math.min(liveTarget, MAX_SAFE_PSI));
        double current = getPressurePsi();

        final double PRESSURE_DEADBAND = 1.0;
        if (current < targetPressure - PRESSURE_DEADBAND) {
            chargeOpen();
            ventClose();
        } else if (current > targetPressure + PRESSURE_DEADBAND) {
            chargeClose();
            ventOpen();
        } else {
            chargeClose();
            ventClose();
        }

        SmartDashboard.putNumber("Cannon/Pressure PSI", current);
    }

    // -------------------- Siren & Strobe --------------------
    private void updateSiren() {
        if (armed) {
            double t = sirenTimer.get() % (SIREN_BURST_TIME * 2);
            sirenTarget = (t < SIREN_BURST_TIME) ? SIREN_MAX_SPEED : 0.0;
        } else {
            sirenTarget = 0.0;
        }

        if (sirenCurrent < sirenTarget)
            sirenCurrent = Math.min(sirenCurrent + SIREN_RAMP_RATE, sirenTarget);
        else if (sirenCurrent > sirenTarget)
            sirenCurrent = Math.max(sirenCurrent - SIREN_RAMP_RATE, sirenTarget);

        sirenMotor.set(sirenCurrent);
    }

    private void updateStrobe() {
        strobeLight.set(armed);
    }

    // -------------------- Periodic --------------------
    @Override
    public void periodic() {
        if (armed) {
            controlPressureLoop();
        } else {
            chargeClose();
            ventClose();
        }

        updateSiren();
        updateStrobe();

        double currentPressure = getPressurePsi();

        // SmartDashboard fallback
        SmartDashboard.putNumber("Cannon/Pressure PSI", currentPressure);
        SmartDashboard.putBoolean("Cannon/Armed", armed);

        // Live Shuffleboard updates
        if (targetPressureEntry != null) targetPressureEntry.setDouble(targetPressure);
        if (currentPressureEntry != null) currentPressureEntry.setDouble(currentPressure);
        if (armedStateEntry != null) armedStateEntry.setBoolean(armed);
    }
}
