package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class LEDSubsystem extends SubsystemBase{

    private static LEDSubsystem INSTANCE;

    public static LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }
    
    private CANdle candle = new CANdle(-1, RobotConstants.CANIVORE_BUS);
    private SolidColor solidColorRequest = new SolidColor(0, 7).withColor(RGBWColor.fromHSV(0, 1, 1));


    public LEDSubsystem() {
        configureCANdle();
    }

    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.LED.StripType = StripTypeValue.RGBW;

        candle.getConfigurator().apply(candleConfig);
    }


    @Override
    public void periodic() {
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

    public RGBWColor getColor() {
        return solidColorRequest.Color;
    }

}
