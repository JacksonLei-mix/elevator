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

public class MotorShooter2 extends SubsystemBase {

    private final TalonFX shooterMotor;

    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Angle> motorPosition;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.05;

    private static final int CURRENT_LIMIT = 70;

    private static final double GEAR_RATIO = 20.0;
    private static final double WHEEL_CIRCUMFERENCE_IN = 4.0;
    public enum MotorshooterStates{
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
    public void loop(){
        MotorshooterIO
    }

    public MotorShooter(int motorID, String canBus) {
        shooterMotor = new TalonFX(motorID, canBus);

        motorCurrent = shooterMotor.getStatorCurrent();
        motorTemp = shooterMotor.getDeviceTemp();
        motorVelocity = shooterMotor.getRotorVelocity();
        motorPosition = shooterMotor.getPosition();

        // Configure motor in Advantage Kit structure
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        shooterMotor.getConfigurator().apply(config);

        // Set update frequency for Advantage Kit signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                50, motorPosition, motorVelocity, motorCurrent, motorTemp);
    }

    /** Set shooter wheel speed in inches per second */
    public void setShooterSpeed(double inchesPerSecond) {
        double wheelRPS = inchesPerSecond / WHEEL_CIRCUMFERENCE_IN;
        double motorRPS = wheelRPS * GEAR_RATIO;
        shooterMotor.setControl(velocityControl.withVelocity(motorRPS));
    }

    /** Stop the shooter */
    public void stop() {
        shooterMotor.stopMotor();
    }

    /** Advantage Kit periodic updates */
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorCurrent, motorTemp);
    }

    /** Telemetry getters */
    public double getMotorCurrent() {
        return motorCurrent.getValue().getValue();
    }

    public double getMotorTemperature() {
        return motorTemp.getValue().getValue(); 
    }

    public double getMotorVelocity() {
        return motorVelocity.getValue().getValue();
    }

    public double getMotorPosition() {
        return motorPosition.getValue().getValue();
    }
}
