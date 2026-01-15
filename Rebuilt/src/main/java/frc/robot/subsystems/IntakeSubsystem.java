package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public final class IntakeSubsystem extends SubsystemBase{
    enum UsableState {
        HOLD,
        INTAKE,
        WAITING,
        SPITOUT
    }
    private UsableState currentState = UsableState.WAITING;
    
    TalonFX Intake_Arm = new TalonFX(5,"CantDrive");
    TalonFX Intake = new TalonFX(6,"CantDrive");
    
    private final MotionMagicVelocityVoltage Intake_Velocity = new MotionMagicVelocityVoltage(0);
    
    private double rps = 0;
    private double arm_angle = 0;
    
    private MotionMagicVoltage request = new MotionMagicVoltage(0);

    
    public void resetEncoders() {
        Intake_Arm.setPosition(0);
    }
    
    private void applyState() {
        switch (currentState) {
            case WAITING:
                arm_angle = 0;
                rps = 0;
            case HOLD:
                arm_angle = 0.5;
                rps = 1;
            case INTAKE:
                arm_angle = 0.5;
                rps = 50;
            case SPITOUT:
                arm_angle = 0.5;
                rps = -50;
        }
    }

    public void configure() {
        //Configure the intake arm
        TalonFXConfiguration ArmConfig = new TalonFXConfiguration();
        ArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ArmConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ArmConfig.CurrentLimits.StatorCurrentLimit = 20;
        ArmConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        ArmConfig.CurrentLimits.SupplyCurrentLimit = 20;
        ArmConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
        ArmConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
        ArmConfig.MotionMagic.MotionMagicAcceleration = 30;
        ArmConfig.MotionMagic.MotionMagicCruiseVelocity = 15;
        ArmConfig.MotionMagic.MotionMagicJerk = 1000;

        var slot0Configs = ArmConfig.Slot0;
        slot0Configs.kS = 0.3;
        slot0Configs.kV = 1.5;
        slot0Configs.kA = 0;
        slot0Configs.kP = 15;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        Intake_Arm.getConfigurator().apply(ArmConfig);

        //Configure intake motors
        TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();
        IntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntakeConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
        IntakeConfig.MotionMagic.MotionMagicAcceleration = 400;
        IntakeConfig.MotionMagic.MotionMagicJerk = 4000;

        var slot0 = IntakeConfig.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 0.11;
        Intake.getConfigurator().apply(IntakeConfig);
    }

    @Override
    public void periodic() {
        applyState();
        Intake.setControl(Intake_Velocity.withVelocity(rps));
        Intake_Arm.setControl(request.withPosition(arm_angle));
    }
}
