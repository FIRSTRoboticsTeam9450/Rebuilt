package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.TrenchManeuvering;
import frc.robot.Constants.FieldConstants.TrenchManeuvering.TrenchLocationWRTBlueOrigin;
import frc.robot.commands.GoToPosCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem.TransferState;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import frc.robot.util.HubTracker;
import frc.robot.util.HubTracker.Shift;

public class CoordinationSubsystem extends SubsystemBase {

    private static volatile CoordinationSubsystem INSTANCE;

    private CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

    public enum AbsoluteStates {
        INIT,
        STORING,
        PREPARING_FOR_SHOOTING,
        SHOOTING,
        TRENCH,
        CLIMB
    }

    private AbsoluteStates currentState = AbsoluteStates.INIT;

    private boolean driverAllowingShooting = false;
    private boolean shouldBeIntaking = false;
    private boolean rejecting = false;
    private boolean autoRejecting = false;
    private boolean fastTransfer = false;
    private boolean intakePushing = false;

    private boolean overrideTrench = false;
    private boolean forceTrench = false;

    private boolean checkZeroing = false;
    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Coord", false);

    public CoordinationSubsystem() {
        
    }


    @Override
    public void periodic() {
        if(checkZeroing) {
            setState(AbsoluteStates.INIT, false);
            checkZeroing = !isZeroingDone();
        }
        
        boolean autoReject = false;// TransferSubsystem.getInstance().checkStall();

        if(autoReject && !autoRejecting) {
            autoRejecting = true;
        } else if (!autoReject && autoRejecting) {
            autoRejecting = false;
        }

        applyLEDs();
        publishLogs();
    }


    private void applyLEDs() {
        LEDState ledState = LEDState.TEAM_UNIQUE_DISABLED;
        double timeRemaining = HubTracker.timeRemainingInCurrentShift().isPresent() ? HubTracker.timeRemainingInCurrentShift().get() : -1;
        if(HubTracker.getCurrentShift().equals(Optional.of(HubTracker.Shift.TRANSITION))) {
            ledState = LEDState.AUTO_STATE;
            if(HubTracker.getAutoWinner().equals(DriverStation.getAlliance())) {
                if(timeRemaining <= 7) {
                    ledState = LEDState.SPEED_STATE;
                    if(timeRemaining <= 2) {
                        ledState = LEDState.END_OF_ACTIVE;
                    }
                }
            }
            else {
                if(timeRemaining <= 7) {
                    ledState = LEDState.COME_BACK;
                    if(timeRemaining <= 2) {
                        ledState = LEDState.END_OF_ACTIVE;
                    }
                }
            }
        } else {
            if(timeRemaining != -1) {
                if(!HubTracker.isActive()) {
                    ledState = LEDState.DEFAULT_INACTIVE;
                    if(timeRemaining <= 10) {
                        ledState = LEDState.COME_BACK;
                        if(timeRemaining <= 5) {
                            ledState = LEDState.END_OF_INACTIVE;
                        }
                    }
                } else {
                    ledState = LEDState.SPEED_STATE;
                    if(timeRemaining <= 3) {
                        ledState = LEDState.END_OF_ACTIVE;
                    }
                }
            }
        }
        if(currentState == AbsoluteStates.TRENCH || currentState == AbsoluteStates.CLIMB) {
            ledState = LEDState.SHOOTING_DEADZONE;
        }
        if(IntakeSubsystem.getInstance().getJammed() || TransferSubsystem.getInstance().jammed()) {
            ledState = LEDState.JAMMED;
        }
        LEDSubsystem.getInstance().setState(ledState);
    }

    private void applyState() {
        TurretState turretState = TurretState.AIMING;
        ShooterState shooterVelocState = ShooterState.NOT_SHOOTING;
        AngleState shooterHoodState = AngleState.AIMING;
        TransferState transferState = TransferState.NO_RUN;
        IntakePos intakeAngleState = IntakePos.STORE;
        IntakeState intakeVelocState = IntakeState.IDLING;
        LEDState ledState = LEDSubsystem.getInstance().getCurrentState();

        switch (currentState) {
            case INIT:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.ZEROING;
                intakeAngleState = IntakePos.STORE;
                intakeVelocState = IntakeState.IDLING;
                ledState = LEDState.INIT;
                break;

            case STORING:
                turretState = TurretState.STORING;
                shooterHoodState = AngleState.STORING;
                if (shouldBeIntaking) {
                    intakeAngleState = IntakePos.INTAKING;
                    intakeVelocState = IntakeState.INTAKING;
                } else if (intakePushing) {
                    intakeAngleState = IntakePos.PUSH_BAR;
                    intakeVelocState = IntakeState.IDLING;
                } else {
                    intakeAngleState = IntakePos.STORE;
                    intakeVelocState = IntakeState.IDLING;
                }
                break;

            case PREPARING_FOR_SHOOTING:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                if (!rejecting && !autoRejecting) {
                    transferState = TransferState.HOLD;
                    if (shouldBeIntaking) {
                        intakeAngleState = IntakePos.INTAKING;
                        intakeVelocState = IntakeState.INTAKING;
                    } else if (intakePushing) {
                        intakeAngleState = IntakePos.PUSH_BAR;
                        intakeVelocState = IntakeState.IDLING;
                    } else {
                        intakeAngleState = IntakePos.STORE;
                        intakeVelocState = IntakeState.IDLING;
                    }
                } else {
                    transferState = TransferState.RUN_OUT;
                    intakeAngleState = IntakePos.STORE;
                    intakeVelocState = IntakeState.IDLING;
                }
                break;

            case SHOOTING:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                if (!rejecting && !autoRejecting) {
                    if (DriverStation.isAutonomous() || !fastTransfer) {
                        transferState = TransferState.RUN_IN;
                    } else {
                        transferState = TransferState.FAST_RUN_IN;
                    }
                    if (shouldBeIntaking) {
                        intakeAngleState = IntakePos.INTAKING;
                        intakeVelocState = IntakeState.INTAKING;
                    } else if (intakePushing) {
                        intakeAngleState = IntakePos.PUSH_BAR;
                        intakeVelocState = IntakeState.IDLING;
                    } else {
                        intakeAngleState = IntakePos.STORE;
                        intakeVelocState = IntakeState.IDLING;
                    }
                } else {
                    transferState = TransferState.RUN_OUT;
                    intakeAngleState = IntakePos.STORE;
                    intakeVelocState = IntakeState.IDLING;
                }
                break;

            case TRENCH:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.STORING;
                shooterVelocState = ShooterState.SHOOTING;
                transferState = TransferState.HOLD;
                // if (TurretSubsystem.getInstance().isAtTotalGoal() && Math.abs(TurretSubsystem.getInstance().getCurrentProfiledSetpoint()) < 1) {
                //     transferState = TransferState.RUN_IN;
                // } else {
                //     transferState = TransferState.HOLD;
                // }
                if (shouldBeIntaking) {
                    intakeAngleState = IntakePos.INTAKING;
                    intakeVelocState = IntakeState.INTAKING;
                } else if (intakePushing) {
                    intakeAngleState = IntakePos.PUSH_BAR;
                    intakeVelocState = IntakeState.IDLING;
                } else {
                    intakeAngleState = IntakePos.TRENCH_STORE;
                    intakeVelocState = IntakeState.IDLING;
                }
                ledState = LEDState.SHOOTING_DEADZONE;
                break;
            case CLIMB:
                turretState = TurretState.CLIMB;
                shooterHoodState = AngleState.STORING;
                shooterVelocState = ShooterState.NOT_SHOOTING;
                transferState = TransferState.NO_RUN;
                
                intakeAngleState = IntakePos.CLIMB_STORE;
                intakeVelocState = IntakeState.IDLING;
                ledState = LEDState.SHOOTING_DEADZONE;
                break;
        }

        TurretSubsystem.getInstance().setState(turretState);
        ShooterSubsystem.getInstance().setState(shooterVelocState, shooterHoodState);
        TransferSubsystem.getInstance().setState(transferState);
        IntakeSubsystem.getInstance().setPos(intakeAngleState);
        IntakeSubsystem.getInstance().setState(intakeVelocState);
        LEDSubsystem.getInstance().setState(ledState);
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Match Data/Our Hub Active?", HubTracker.isActive());
        Logger.recordOutput("Rebuilt/Match Data/We Win Auto?", DriverStation.getAlliance().isPresent() && HubTracker.getAutoWinner().isPresent() ? DriverStation.getAlliance().get() == HubTracker.getAutoWinner().get() : false);
        Logger.recordOutput("Rebuilt/Match Data/Time Left Until Shift Change", HubTracker.timeRemainingInCurrentShift().isPresent() ? HubTracker.timeRemainingInCurrentShift().get() : -1);
        Logger.recordOutput("Rebuilt/Match Data/Shift", HubTracker.getCurrentShift().isPresent() ? HubTracker.getCurrentShift().get() : Shift.AUTO);

        Logger.recordOutput("Rebuilt/Coord/Intakeing", shouldBeIntaking);
        Logger.recordOutput("Rebuilt/Coord/Firing", driverAllowingShooting);

        // Logger.recordOutput("Rebuilt/Coord/Current State", currentState);
        // Logger.recordOutput("Rebuilt/Coord/Driver Allows Shooting", driverAllowingShooting);
        // Logger.recordOutput("Rebuilt/Coord/Subsystems good to fire?", areShooterAndTurretReady());
        // Logger.recordOutput("Rebuilt/Coord/Is Shooter Ready?", isShooterReady());
        // Logger.recordOutput("Rebuilt/Coord/Is Turret Ready?", isTurretReady());
        // Logger.recordOutput("Rebuilt/Coord/OverridingTrench?", overrideTrench);

        if(logDebuggingToggle.get()) {
        }
    }

    // ======================== Conditions =============================
    public boolean isZeroingDone() {
        boolean zeroingDone = ShooterSubsystem.getInstance().getCurrentAngleState() != AngleState.ZEROING 
        && IntakeSubsystem.getInstance().getCurrentIntakePos() != IntakePos.ZEROING;
        if(zeroingDone) {
            setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
            checkZeroing = false;
        }
        return zeroingDone;
    }

    public boolean isDriverWantingToShoot() {
        return driverAllowingShooting;
    }

    public boolean goodToFire() {
        return driverAllowingShooting;
    }

    public boolean isFastTransfer() {
        return fastTransfer;
    }

    public Command setFireCommand() {
        return new InstantCommand(() -> driverAllowingShooting = !driverAllowingShooting);
    }

    public Command setFireCommand(boolean value) {
        return new InstantCommand(() -> driverAllowingShooting = value);
    }

    public Command setOverrideCommand(boolean value) {
        return new InstantCommand(() -> overrideTrench = value);
    }

    public Command setIntakeCommand() {
        return new InstantCommand(() -> shouldBeIntaking = !shouldBeIntaking)
            .andThen(new InstantCommand(() -> CoordinationSubsystem.getInstance().applyState()));
    }

    public Command setIntakeCommand(boolean value) {
        return new InstantCommand(() -> shouldBeIntaking = value)
            .andThen(new InstantCommand(() -> CoordinationSubsystem.getInstance().applyState()));
    }

    public Command setRejectingCommand(boolean value) {
        return new InstantCommand(() -> rejecting = value);
    }

    public Command setFastTransferCommand(boolean value) {
        return new InstantCommand(() -> fastTransfer = value);
    }

    public Command setFastTransferCommand() {
        return new InstantCommand(() -> fastTransfer = !fastTransfer);
    }

    public Command setIntakePushing(boolean value) {
        return new InstantCommand(() -> intakePushing = value);
    }

    public Command setIntakePushing() {
        return new InstantCommand(() -> intakePushing = !intakePushing);
    }

    public Command setHardTrenchOverride(boolean value) {
        return new InstantCommand(() -> currentState = AbsoluteStates.PREPARING_FOR_SHOOTING)
        .andThen(new InstantCommand(() -> CoordinationSubsystem.getInstance().applyState()));
    }

    public Command setForceTrenchCommand() {
        return new InstantCommand(() -> forceTrench = !forceTrench);
    }

    public Command increaseManualAdjust() {
        return new InstantCommand(() -> ShooterSubsystem.getInstance().addManualVelocityShift());
    }
    public Command increaseManualAdjust(double value) {
        return new InstantCommand(() -> ShooterSubsystem.getInstance().addManualVelocityShift(value));
    }
    public Command decreaseManualAdjust() {
        return new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualVelocityShift());
    }
    public Command decreaseManualAdjust(double value) {
        return new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualVelocityShift(value));
    }
    public Command resetManualAdjust() {
        return new InstantCommand(() -> ShooterSubsystem.getInstance().resetManualVelocityShift());
    }

    // ============================ Getters & Setters ============================

    public AbsoluteStates getCurrentAbsoluteState() {
        return currentState;
    }

    public Command setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin location) {
        switch (location) {
            case CLOSE_LEFT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchOne, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchTwo, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchEight, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSeven, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            (() -> FieldConstants.isBlueAlliance)
                        );


            case CLOSE_RIGHT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchThree, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFour, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSix, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFive, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            (() -> FieldConstants.isBlueAlliance)
                        );

            case FAR_LEFT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFive, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSix, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFour, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchThree, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            (() -> FieldConstants.isBlueAlliance)
                        );

            case FAR_RIGHT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSeven, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchEight, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchTwo, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchOne, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            (() -> FieldConstants.isBlueAlliance)
                        );
        }

    return new GoToPosCommand(drivetrain.getState().Pose);
    }
    public boolean getOverridingTrench() {
        if(overrideTrench) {
            return true;
        }
        return false;
    }

    public void setState(AbsoluteStates state, boolean forceStateChange) {
        if (currentState != AbsoluteStates.CLIMB || forceStateChange) {
            currentState = state;
            applyState();
        }
    }

    public Command setStateCommand(AbsoluteStates state, boolean forceStateChange) {
        return new InstantCommand(() -> setState(state, forceStateChange));
    }

    // ======================= INIT ===================

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CoordinationSubsystem();
            INSTANCE.initizalize();
        }
        return INSTANCE;
    }

    public void initizalize() {
    }
}
