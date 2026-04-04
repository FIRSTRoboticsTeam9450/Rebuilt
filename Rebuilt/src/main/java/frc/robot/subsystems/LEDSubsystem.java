package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.LEDConstants;
import frc.robot.commands.TeamCANdleAnimationCommand;
import frc.robot.util.HubTracker;

/**
 * I WANT MY LEDs!!!!!!! <br> <br/>
 * Currently Disabled due to not having weight on robot. （；´д｀）ゞ <br><br/>
 * No longer disabled :D
 */
public class LEDSubsystem extends SubsystemBase{
    private static volatile LEDSubsystem INSTANCE;
    
    public static final int HIGHEST_INDEX = 107;
    public static final int START_INDEX = 0;
    public static final int END_INDEX = 18;
    public static final RGBWColor EMPTY_COLOR = new RGBWColor(0, 0, 0);

    private int amountOfApplyStates = 0;

    private TeamCANdleAnimationCommand teamUniqueDisabledCommand = new TeamCANdleAnimationCommand(this);

    private CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private double intakeWallMax = 0;
    private double intakeWallMin = 0;

    public static synchronized LEDSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LEDSubsystem();
            Initialize();
        }
        return INSTANCE;
    }

    private static void Initialize() {
        INSTANCE.applyState();
    }

    public enum LEDState {
        SYS_INIT,
        TEAM_UNIQUE_DISABLED,
        INIT,
        AUTO_STATE, // If win: Flashing Bright green for 3 secs then go to active state If lose: Flashing bright red for 3 secs then go to inactive state
        COME_BACK, // Solid white
        END_OF_INACTIVE, // Flashing white for 5 secs, (one pulse per sec)
        DEFAULT_INACTIVE,
        JAMMED, // Flashing pink
        END_OF_ACTIVE, // Flashing white for 3 secs
        SPEED_STATE, // Default purple if going perfect speed for intaking a wall of balls
        SHOOTING_DEADZONE // Flashing yellow
    }

    enum SpeedState {
        TOO_FAST,
        GOOD,
        TOO_SLOW
    }

    private LEDState currentState = LEDState.SYS_INIT;
    private SpeedState currentSpeedState = SpeedState.GOOD;
    
    private CANdle candle = new CANdle(LEDConstants.DeviceIDs.kCANDLEID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private SolidColor solidColorRequest = new SolidColor(START_INDEX, END_INDEX);
    private EmptyAnimation emptyColorRequest = new EmptyAnimation(0);

    private ColorFlowAnimation flowColorRequest = new ColorFlowAnimation(START_INDEX, END_INDEX).withFrameRate(10);
    private StrobeAnimation strobeColorRequest = new StrobeAnimation(START_INDEX, END_INDEX).withFrameRate(2); // Fast
    private StrobeAnimation slowStrobeColorRequest = new StrobeAnimation(START_INDEX, END_INDEX).withFrameRate(5); // Slow
    private SingleFadeAnimation singleFadeRequest = new SingleFadeAnimation(START_INDEX, END_INDEX).withFrameRate(25);

    public LEDSubsystem() {
        configureCANdle();
    }

    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        candleConfig.LED.BrightnessScalar = 1;
        candleConfig.LED.StripType = StripTypeValue.RGB;
        
        candle.getConfigurator().apply(candleConfig);
    }


    @Override
    public void periodic() {
        if (currentState == LEDState.SPEED_STATE) { 
            SpeedState checkedSpeedState = SpeedState.GOOD;

            double xVeloc = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
            double yVeloc = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;
            double totalVeloc = Math.hypot(xVeloc, yVeloc);

            if (totalVeloc >= 0.55 && totalVeloc <= 0.75) {
                checkedSpeedState = SpeedState.GOOD;
            } else if (totalVeloc > 0.75) {
                checkedSpeedState = SpeedState.TOO_FAST;
            } else if (totalVeloc < 0.55) {
                checkedSpeedState = SpeedState.TOO_SLOW;
            }
            if (currentSpeedState != checkedSpeedState) {
                currentSpeedState = checkedSpeedState;
                applyState();
            }
        }
        publishLogs();
    }

    @SuppressWarnings("static-access")
    private void applyState() {
        amountOfApplyStates++;
        switch (currentState) {
            case SYS_INIT:
                candle.setControl(new LarsonAnimation(START_INDEX, END_INDEX).withColor(RGBWColor.fromHSV(30, 1, 1)).withFrameRate(20).withSize(5));
                break;

            case TEAM_UNIQUE_DISABLED:
                CommandScheduler.getInstance().schedule(teamUniqueDisabledCommand);
                candle.setControl(new SingleFadeAnimation(START_INDEX, END_INDEX).withColor(RGBWColor.fromHSV(240, 1, 1)).withFrameRate(100));
                break;

            /*case DISABLED:
                break;*/
            
            case INIT:
                candle.setControl(new StrobeAnimation(START_INDEX, END_INDEX).withColor(RGBWColor.fromHSV(15, 1, 1)).withFrameRate(3));
                break;
            
            case AUTO_STATE: // Green - H=115, Red - H=0
                if (HubTracker.getAutoWinner().equals(DriverStation.getAlliance()))
                    candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(115, 1, 1)));
                else
                    candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(0, 1, 1)));
                break;

            case DEFAULT_INACTIVE: // Nothing
                removeLEDColor(true);
                break;

            case COME_BACK: // White - S=0
                candle.setControl(solidColorRequest.withColor(RGBWColor.fromHSV(0, 0, 1)));
                break;

            case END_OF_INACTIVE: // White - S=0
                candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(0, 0, 1)));
                break;

            case JAMMED: // Pink - H=355
                candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(355, 1, 1)));
                break;

            case END_OF_ACTIVE: // White - S=0
                candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(0, 0, 1)));
                break;
            
            case SPEED_STATE: // Purple - H=310
                if (currentSpeedState == SpeedState.GOOD) {
                    setColor(310, 1, 1);
                    applyColorControl();
                } else if (currentSpeedState == SpeedState.TOO_FAST) {
                    candle.setControl(new StrobeAnimation(START_INDEX, END_INDEX).withColor(new RGBWColor().fromHSV(310, 1, 1)).withFrameRate(6));
                } else if (currentSpeedState == SpeedState.TOO_SLOW) {
                    candle.setControl(new StrobeAnimation(START_INDEX, END_INDEX).withColor(new RGBWColor().fromHSV(310, 1, 1)).withFrameRate(1));
                }
                break;

            case SHOOTING_DEADZONE: // Yellow - H=30
                candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(30, 1, 1)).withFrameRate(3));
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/LED/Current State", currentState);
        Logger.recordOutput("Rebuilt/LED/Current Control Request", candle.getAppliedControl().toString());
        Logger.recordOutput("Rebuilt/LED/Amount of Apply State Calls", amountOfApplyStates);
    }

    public void removeLEDColor(boolean cancelUniqueAnimation) {
        if (cancelUniqueAnimation) CommandScheduler.getInstance().cancel(teamUniqueDisabledCommand);

        candle.setControl(emptyColorRequest);
        candle.setControl(solidColorRequest.withColor(EMPTY_COLOR).withLEDStartIndex(0).withLEDEndIndex(HIGHEST_INDEX));
    }

    public void setState(LEDState state) {
        if (currentState == state) return;
        currentState = state;
        removeLEDColor(true);
        applyState();
    }

    public Command setStateCommand(LEDState state) {
        return new InstantCommand(() -> setState(state));
    }

    public void applyColorControl() {
        candle.setControl(solidColorRequest);
    }

    /**
     * Changes the color that the CANdle sets the LEDs to.
     * @param h Hue, range of: { 0-360 }
     * @param s Saturation, range of: { 0-1 }
     * @param v Value, range of: { 0-1 }
     */
    public void setColor(double h, double s, double v) {
        h = MathUtil.clamp(h, 0, 360);
        s = MathUtil.clamp(s, 0, 1);
        v = MathUtil.clamp(v, 0, 1);
        solidColorRequest.Color = RGBWColor.fromHSV(h, s, v);
    }
    
    public void setColor(RGBWColor color) {
        solidColorRequest.Color = color;
    }

    public RGBWColor getColor() {
        return solidColorRequest.Color;
    }

    public void setColorStart(int index) {
        if (index < 0) index = 0; if (index > END_INDEX) index = END_INDEX;
        solidColorRequest.LEDStartIndex = index;
    }
    public void setColorEnd(int index) {
        if (index < 0) index = 0; if (index > END_INDEX) index = END_INDEX;
        solidColorRequest.LEDEndIndex = index;
    }
    public int getColorStart() {
        return solidColorRequest.LEDStartIndex;
    }
    public int getColorEnd() {
        return solidColorRequest.LEDEndIndex;
    }
    public void setColorRange(int startIndex, int endIndex) {
        setColorStart(startIndex);
        setColorEnd(endIndex);
    }
}
