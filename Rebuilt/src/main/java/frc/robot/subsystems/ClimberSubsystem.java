package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.TurretSubsystem.UsableStates;

public class ClimberSubsystem extends SubsystemBase {

    private static ClimberSubsystem climber;

    private double voltage;
    private double maxVoltage;

    private double offset;

    private TalonFX climbMotor = new TalonFX(ClimberConstants.MotorIDs.kElevatorMotorID, RobotConstants.CANIVORE_BUS_NAME);

    private CANcoder climbEncoder = new CANcoder(ClimberConstants.MotorIDs.kElevatorMotorID, RobotConstants.CANIVORE_BUS_NAME);

    private LoggedNetworkNumber logMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/Velocity", 0.1);
    private LoggedNetworkNumber logMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/Velocity", 1);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Climb/Velocity", 1000);
    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Climb/kS", 0.24);
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Climb/kV", 0.33);
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Climb/kS", 0.1);
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Climb/kS", 0.5);
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Climb/kS", 0.0);
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Climb/kS", 0.0);

    private TalonFXConfiguration climbConfig;

    private DynamicMotionMagicVoltage controlRequest = new DynamicMotionMagicVoltage(0, logMMVelocity.get(), logMMAcceleration.get()).withJerk(logMMJerk.get())
                                                                                                                                              .withSlot(0);
    // MotionMagic type may change

    public enum UsableStates{
        Zeroing,
        Climb
    };

    private UsableStates currentState = UsableStates.Zeroing;

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        configureMotors();
    }

    /* ----------- Updaters ----------- */
    public void configureMotors() {
        climbConfig = new TalonFXConfiguration();

        climbConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                                      .withKS(logMMKS.get())
                                                      .withKV(logMMKV.get())
                                                      .withKA(logMMKA.get())
                                                      .withKP(logMMKP.get())
                                                      .withKI(logMMKI.get())
                                                      .withKD(logMMKD.get());

        climbConfig.Slot0 = slot0Configs;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        climbConfig.MotionMagic = mmConfigs;

        climbMotor.getConfigurator().apply(climbConfig);
    }
    
    public void setVoltage(double voltage) {}


    public void setMaxVoltage(double maxVoltage) {
        this.maxVoltage = 12; //will be changed to maxVoltage
    }

    public void setPoint(double setpoint) {
        climbMotor.setControl(controlRequest.withPosition(setpoint));
    }


    public void resetEncoders() {
        climbMotor.setVoltage(0);
        climbMotor.setPosition(0);
    }

    public void periodic() {
        applyStates();
    }

    public void applyStates() {
        switch (currentState) {
            case Zeroing:
                climbMotor.setVoltage(-1);
                if (climbMotor.getMotorVoltage().getValueAsDouble() > 0.8 && climbMotor.getVelocity().getValueAsDouble() < 0.1) {
                    climbMotor.setVoltage(0);
                    climbMotor.setPosition(0);
                    currentState = UsableStates.Climb;
                }
                break;
        
            default:
                break;
        }
    }

    public double getPosition() {
        return climbEncoder.getPosition().getValueAsDouble();
    } //change return type to "double" after encoders are instantiated

    public static ClimberSubsystem getInstance() {
        if (climber == null) {
            climber = new ClimberSubsystem();
        }

        return climber;
    }
    
}
