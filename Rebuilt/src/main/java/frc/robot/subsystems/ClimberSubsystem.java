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
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private double setpoint;
    private double offset;

    private TalonFX climbMotor = new TalonFX(ClimberConstants.MotorIDs.kElevatorMotorID, RobotConstants.CANIVORE_BUS_NAME);
    PIDController pid = new PIDController(0, 0, 0);

    private TalonFXConfiguration climbConfig;

    MotionMagicConfigs climbConfigs = new MotionMagicConfigs();
    // MotionMagic type may change

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        
        Slot0Configs slot0Configs = climbConfig.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = climbConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        climbMotor.getConfigurator().apply(climbConfig);
    }

    /* ----------- Updaters ----------- */
    
    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
        this.setpoint = setpoint;
        climbMotor.setControl(new DynamicMotionMagicVoltage(null, null, null).withPosition(setpoint + offset - 0.25));
    }

    public void setMaxVoltage(double maxVoltage) {
        this.maxVoltage = 12;
    }

    public void updatePIDs() {}

    public void resetEncoders() {}

    public void periodic() {}

    /*
    public double getPosition() {
        return climbEncoder.getPosition().getValueAsDouble();
    }
    */

    public static ClimberSubsystem getInstance() {
        if (climber == null) {
            climber = new ClimberSubsystem();
        }

        return climber;
    }
    
}
