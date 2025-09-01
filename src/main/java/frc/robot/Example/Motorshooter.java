package frc.robot.Example;

import java.security.Key;
import java.util.Currency;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motorshooter extends SubsystemBase{
    private final TalonFX shooterTalonFX;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    //PID Control
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kF = 0.05;
    private static final double kD = 0.0;

    private static final int CURRENT_LIMIT = 70;//amps
    private static final double GEAR_RATIO = 20.0;
    private static final double WHEEL_CIRCUMFERENCE_IN = 4.0;//give 4.0 inches;
     
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<AngularVelocity> motorAngularVelocity;
    private final StatusSignal<Angle> motorAngle;
    public Motorshooter(int shooterID, String canBus){
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
        config.Slot0.kV = kF;//apply PID;

        shooterTalonFX.getConfigurator().apply(config);
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorAngle,motorAngularVelocity,motorCurrent,motorTemp);
    }
    public void setShooterSpeed(double inchesPerSecond){
        double wheelRotationPersecond = inchesPerSecond/WHEEL_CIRCUMFERENCE_IN;
        double motorRPS = wheelRotationPersecond/GEAR_RATIO;//covert  inches/s --> motorRPS(ratio)
        shooterTalonFX.setControl(velocityRequest.withVelocity(motorRPS)); //load RPS;

    }
    //stop function(maybe useless)
    public void stop() {
        shooterTalonFX.stopMotor();
    }
    public void periodic(){
        BaseStatusSignal.refreshAll(motorAngle,motorAngularVelocity,motorCurrent,motorTemp);

    }


}