package frc.robot.subsystems.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.VectorTools.HSV;
import frc.robot.LEDConstants;
import com.ctre.phoenix.CANifier;

/*
 * Orange dot
 */
public class OrangeDot extends LEDModeBase {
    private double m_orangeDotMiddleIndex = -LEDConstants.OrangeDot.pauseBetween;
    private HSV hsv = LEDConstants.OrangeDot.hsv;

    private Color colour = LEDConstants.OrangeDot.rgb;

    public OrangeDot(AddressableLEDBuffer m_ledBuffer) {
        super(m_ledBuffer);
    }

    public void execute() {
        for (int i = 0; i < m_ledBuffer.getLength() + LEDConstants.OrangeDot.pauseBetween; i++) {
            int value =
                    MathUtil.clamp(
                                    (int)
                                                    ((1 / LEDConstants.OrangeDot.spread)
                                                            * (Math.abs(m_orangeDotMiddleIndex - i))
                                                            * hsv.v)
                                            - LEDConstants.OrangeDot.length,
                                    0,
                                    hsv.v)
                            - hsv.v;

            if (m_ledBuffer.getLength() > i) m_ledBuffer.setHSV(i, hsv.h, hsv.s, value);
        }
        m_orangeDotMiddleIndex =
                (m_orangeDotMiddleIndex + LEDConstants.OrangeDot.speed)
                                > m_ledBuffer.getLength() + LEDConstants.OrangeDot.pauseBetween
                        ? -LEDConstants.OrangeDot.pauseBetween
                        : (m_orangeDotMiddleIndex + LEDConstants.OrangeDot.speed);
    }
}
