package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.LEDConstants;
import frc.robot.commands.TeamCANdleAnimationCommand;

/**
 * I WANT MY LEDs!!!!!!!
 */
public class LEDSubsystem extends SubsystemBase{
    private static volatile LEDSubsystem INSTANCE;
    
    public static final int HIGHEST_INDEX = 107;
    public static final int START_INDEX = 0;
    public static final int END_INDEX = 14;
    public static final RGBWColor EMPTY_COLOR = new RGBWColor(0, 0, 0);

    // private TeamCANdleAnimationCommand teamUniqueDisabledCommand;

    public static synchronized LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }

    public enum LEDState {
        TEAM_UNIQUE_DISABLED,
        DISABLED,
        INIT,
        STORING,
        UNABLE_TO_FIRE,
        PREPARING_TO_FIRE,
        GOOD_TO_FIRE,
        DEADZONE_OVERRIDEN
    }

    private LEDState currentState = LEDState.DISABLED;
    
    private CANdle candle = new CANdle(LEDConstants.DeviceIDs.kCANDleID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private SolidColor solidColorRequest = new SolidColor(START_INDEX, END_INDEX);
    private EmptyAnimation emptyColorRequest = new EmptyAnimation(0);

    private ColorFlowAnimation flowColorRequest = new ColorFlowAnimation(START_INDEX, END_INDEX);
    private StrobeAnimation strobeColorRequest = new StrobeAnimation(START_INDEX, END_INDEX);
    private SingleFadeAnimation singleFadeRequest = new SingleFadeAnimation(START_INDEX, END_INDEX);

    public LEDSubsystem() {
        configureCANdle();

        // teamUniqueDisabledCommand = new TeamCANdleAnimationCommand(INSTANCE);
    }

    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candleConfig.LED.StripType = StripTypeValue.RGB;

        candle.getConfigurator().apply(candleConfig);
    }


    @Override
    public void periodic() {
        if (currentState == LEDState.DISABLED && !DriverStation.isFMSAttached()) {
            setState(LEDState.TEAM_UNIQUE_DISABLED);
        }
        publishLogs();
    }

    private void applyState() {
        switch (currentState) {
            case TEAM_UNIQUE_DISABLED:
                // CommandScheduler.getInstance().schedule(teamUniqueDisabledCommand);
                break;

            case DISABLED:
                candle.setControl(singleFadeRequest.withColor(RGBWColor.fromHSV(45, 1, 1)).withFrameRate(25));
                break;
            
            case INIT:
                candle.setControl(flowColorRequest.withColor(RGBWColor.fromHSV(60, 1, 1)).withFrameRate(10));
                break;

            case STORING:
                candle.setControl(solidColorRequest.withColor(RGBWColor.fromHSV(0, 0, 1)));
                break;

            case UNABLE_TO_FIRE:
                candle.setControl(solidColorRequest.withColor(RGBWColor.fromHSV(0, 1, 1)));
                break;

            case PREPARING_TO_FIRE:
                candle.setControl(solidColorRequest.withColor(RGBWColor.fromHSV(60, 1, 1)));
                break;

            case GOOD_TO_FIRE:
                candle.setControl(solidColorRequest.withColor(RGBWColor.fromHSV(120, 1, 1)));
                break;

            case DEADZONE_OVERRIDEN:
                candle.setControl(strobeColorRequest.withColor(RGBWColor.fromHSV(0, 1, 1)).withFrameRate(5));
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/LED/Current State", currentState);
        Logger.recordOutput("Rebuilt/LED/Current Control Request", candle.getAppliedControl().toString());
    }

    public void removeLEDColor() {
        // if (teamUniqueDisabledCommand.isScheduled()) {
        //     CommandScheduler.getInstance().cancel(teamUniqueDisabledCommand);
        // }
        candle.setControl(emptyColorRequest);
        candle.setControl(solidColorRequest.withColor(EMPTY_COLOR).withLEDStartIndex(0).withLEDEndIndex(HIGHEST_INDEX));
    }

    public void setState(LEDState state) {
        currentState = state;
        removeLEDColor();
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
        if (index < 0) index = 0; if (index > HIGHEST_INDEX) index = HIGHEST_INDEX;
        solidColorRequest.LEDStartIndex = index;
    }
    public void setColorEnd(int index) {
        if (index < 0) index = 0; if (index > HIGHEST_INDEX) index = HIGHEST_INDEX;
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
