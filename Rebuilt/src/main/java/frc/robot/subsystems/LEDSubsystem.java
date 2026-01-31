package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Animation0TypeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.LEDConstants;

public class LEDSubsystem extends SubsystemBase{

    private static volatile LEDSubsystem INSTANCE;

    public static synchronized LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }

    public enum LEDState {
        DISABLED,
        INIT,
        STORING,
        UNABLE_TO_FIRE,
        PREPARING_TO_FIRE,
        GOOD_TO_FIRE
    }

    private LEDState currentState = LEDState.DISABLED;
    
    private CANdle candle = new CANdle(LEDConstants.DeviceIDs.kCANDleID, RobotConstants.CANIVORE_BUS);
    private SolidColor solidColorRequest = new SolidColor(0, 14);
    private ColorFlowAnimation flowColorRequest = new ColorFlowAnimation(0, 14);
    private StrobeAnimation strobeColorRequest = new StrobeAnimation(0, 14);
    private SingleFadeAnimation singleFadeRequest = new SingleFadeAnimation(0, 14);

    public LEDSubsystem() {
        configureCANdle();
    }

    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.LED.StripType = StripTypeValue.RGB;

        candle.getConfigurator().apply(candleConfig);
    }


    @Override
    public void periodic() {
        // applyState();

        publishLogs();
    }

    private void applyState() {
        // candle.setControl(solidColorRequest.withColor(RGBWColor.fromHSV(0, 0, 0)));
        switch (currentState) {
            case DISABLED:
                candle.setControl(singleFadeRequest.withColor(RGBWColor.fromHSV(45, 1, 1)).withSlot(0));
                break;
            
            case INIT:
                candle.setControl(flowColorRequest.withColor(RGBWColor.fromHSV(60, 1, 1)).withSlot(1));
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
        }
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/LED/Current State", currentState);
        Logger.recordOutput("Rebuilt/LED/Current Control Request", candle.getAppliedControl().toString());
    }

    public void setState(LEDState state) {
        currentState = state;

        applyState();
    }

    public Command setStateCommand(LEDState state) {
        return new InstantCommand(() -> setState(state));
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

    public RGBWColor getColor() {
        return solidColorRequest.Color;
    }

}
