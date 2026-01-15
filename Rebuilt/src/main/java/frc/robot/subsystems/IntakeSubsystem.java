package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public final class IntakeSubsystem extends SubsystemBase{
    //List the states the intake will use
    enum UsableState {
        HOLD,
        INTAKE,
        WAITING,
        SPITOUT
    }

    //Set the current state of the robot to be idle
    private UsableState currentState = UsableState.WAITING;
    
    //Set up motors
    TalonFX Intake_Arm = new TalonFX(5,"CantDrive");
    TalonFX Intake = new TalonFX(6,"CantDrive");
    
    //Velocity Motion Magic set up (Speed of intake rollers)
    private final MotionMagicVelocityVoltage Intake_Velocity = new MotionMagicVelocityVoltage(0);

    //Set up the rps of the intake motors to 0
    private double rps = 0;

    //Motion Magic Set Up(Intake arm)
    private MotionMagicVoltage request = new MotionMagicVoltage(0);

    //Set the arm to 0
    private double arm_angle = 0;

    public IntakeSubsystem() {
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

        //Magic Motion
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

        //Magic Velocity for rollers
        var slot0 = IntakeConfig.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 0.11;
        Intake.getConfigurator().apply(IntakeConfig);
    }
    
    public void resetEncoders() {
        //Sets the arm to 0
        Intake_Arm.setPosition(0);
    }
    
    private void setState(UsableState state) {
        currentState = state;
        switch (currentState) {
            case WAITING:
                arm_angle = 0;
                rps = 0;
                break;
            case HOLD:
                arm_angle = 0.5;
                rps = 1;
                break;
            case INTAKE:
                arm_angle = 0.5;
                rps = 50;
                break;
            case SPITOUT:
                arm_angle = 0.5;
                rps = -50;
                break;
        }
    }

    @Override
    public void periodic() {
        //Sets the intake and arm to specified amounts
        Intake.setControl(Intake_Velocity.withVelocity(rps));
        Intake_Arm.setControl(request.withPosition(arm_angle));
    }
}
