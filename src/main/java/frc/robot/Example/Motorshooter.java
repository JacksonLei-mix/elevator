package frc.robot.Example;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motorshooter extends SubsystemBase {
    private final TalonFX shooterTalonFX;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    // PID Control
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kF = 0.05;
    private static final double kD = 0.0;

    private static final int CURRENT_LIMIT = 70; // amps
    private static final double GEAR_RATIO = 20.0;
    private static final double WHEEL_CIRCUMFERENCE_IN = 4.0; // inches
     
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<AngularVelocity> motorAngularVelocity;
    private final StatusSignal<Angle> motorAngle;

    public enum MotorshooterStates {
        IDLE,
        ZERO,
        HANDOFF,
        SHOOT,
        AMP,
        PASS,
        VOLTAGE,
        OUTTAKE,
        VELOCITY
    }

    // current state
    private MotorshooterStates currentState = MotorshooterStates.IDLE;

    public Motorshooter(int shooterID, String canBus) {
        shooterTalonFX = new TalonFX(shooterID, canBus);
        motorCurrent = shooterTalonFX.getStatorCurrent();
        motorTemp = shooterTalonFX.getDeviceTemp();
        motorAngularVelocity = shooterTalonFX.getRotorVelocity();
        motorAngle = shooterTalonFX.getPosition();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        shooterTalonFX.getConfigurator().apply(config);
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorAngle, motorAngularVelocity, motorCurrent, motorTemp);
    }

    public void setShooterSpeed(double inchesPerSecond) {
        double wheelRotationPersecond = inchesPerSecond / WHEEL_CIRCUMFERENCE_IN;
        double motorRPS = wheelRotationPersecond / GEAR_RATIO;
        shooterTalonFX.setControl(velocityRequest.withVelocity(motorRPS));
    }

    public void stop() {
        shooterTalonFX.stopMotor();
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(motorAngle, motorAngularVelocity, motorCurrent, motorTemp);

        // run state machine logic
        switch (currentState) {
            case IDLE:
                stop();
                break;
            case ZERO:
                // for example: reset encoder
                shooterTalonFX.setPosition(0);
                currentState = MotorshooterStates.IDLE;
                break;
            case HANDOFF:
                setShooterSpeed(20.0); // low speed
                break;
            case SHOOT:
                setShooterSpeed(100.0); // high speed for shooting
                break;
            case AMP:
                setShooterSpeed(30.0);
                break;
            case PASS:
                setShooterSpeed(50.0);
                break;
            case VOLTAGE:
                shooterTalonFX.set(0.5); // half voltage
                break;
            case OUTTAKE:
                setShooterSpeed(-40.0); // reverse
                break;
            case VELOCITY:
                setShooterSpeed(60.0); // example constant velocity
                break;
        }
    }

    // public setter for state
    public void setState(MotorshooterStates newState) {
        currentState = newState;
    }

    // getter for state
    public MotorshooterStates getState() {
        return currentState;
    }
}
