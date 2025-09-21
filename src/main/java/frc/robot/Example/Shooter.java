package frc.robot.Example;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // PIDF gains
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.05; // feedforward

    // Motor constants
    private static final int CURRENT_LIMIT = 70; // amps
    private static final double GEAR_RATIO = 20.0;
    private static final double WHEEL_CIRCUMFERENCE_IN = 4.0; // inches

    // Signals
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<AngularVelocity> motorAngularVelocity;
    private final StatusSignal<Angle> motorAngle;

    // State machine
    public enum ShooterState {
        IDLE, ZERO, HANDOFF, SHOOT, AMP, PASS, VOLTAGE, OUTTAKE, VELOCITY
    }

    private ShooterState currentState = ShooterState.IDLE;

    public Shooter(int shooterID, String canBus) {
        shooterMotor = new TalonFX(shooterID, canBus);

        motorCurrent = shooterMotor.getStatorCurrent();
        motorTemp = shooterMotor.getDeviceTemp();
        motorAngularVelocity = shooterMotor.getRotorVelocity();
        motorAngle = shooterMotor.getPosition();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;

        shooterMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, motorAngle, motorAngularVelocity, motorCurrent, motorTemp
        );
    }

    /** Convert target linear speed (inches per second) to motor RPS and set */
    public void setTargetSpeedIPS(double inchesPerSecond) {
        double wheelRPS = inchesPerSecond / WHEEL_CIRCUMFERENCE_IN;
        double motorRPS = wheelRPS / GEAR_RATIO;
        shooterMotor.setControl(velocityRequest.withVelocity(motorRPS));
    }

    public void stop() {
        shooterMotor.stopMotor();
    }

    public void setState(ShooterState newState) {
        this.currentState = newState;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorAngle, motorAngularVelocity, motorCurrent, motorTemp);

        switch (currentState) {
            case IDLE:
                stop();
                break;

            case ZERO:
                // Example: move motor slowly until limit switch or encoder reset
                shooterMotor.setControl(voltageRequest.withOutput(-1.0));
                break;

            case HANDOFF:
                // Low speed to transfer note from intake to shooter
                setTargetSpeedIPS(20.0);
                break;

            case SHOOT:
                // High shooter speed for scoring
                setTargetSpeedIPS(150.0);
                break;

            case AMP:
                // Slower speed tuned for AMP shot
                setTargetSpeedIPS(60.0);
                break;

            case PASS:
                // Medium speed to pass the note
                setTargetSpeedIPS(80.0);
                break;

            case VOLTAGE:
                // Open-loop control (example: 6 volts)
                shooterMotor.setControl(voltageRequest.withOutput(6.0));
                break;

            case OUTTAKE:
                // Reverse shooter to eject piece
                setTargetSpeedIPS(-50.0);
                break;

            case VELOCITY:
                // Placeholder: use last commanded velocity
                // Could add logic here to hold current speed
                break;
        }
    }
}
