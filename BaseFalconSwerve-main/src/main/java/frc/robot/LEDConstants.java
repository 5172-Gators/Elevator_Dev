package frc.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;

import frc.lib.VectorTools.HSV;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int id = 17;
    public static final int length = 18;

    public static final LEDMode defaultMode = LEDMode.Idle;

    public static final class Flash {
        public static final double speed = 5;
    }

    public static final class IdleMode {
        public static final Color rgb = Color.kFirstBlue;

        public static final int pauseBetween = 10;
        public static final int length = 30;
        public static final double spread = 4;
        public static final double speed = .25;
    }

    public static final class HumanPlayerCube {
        public static final Color rgb = Color.kDarkViolet;

        public static final int pauseBetween = 5;
        public static final int length = 0;
        public static final double spread = 2;
        public static final double speed = .5;
    }

    public static final class HumanPlayerCone {
        public static final Color rgb = Color.kGold;

        public static final int pauseBetween = 5;
        public static final int length = 0;
        public static final double spread = 2;
        public static final double speed = .5;
    }

    public enum LEDMode {
        HumanPlayerCube("Cube Signal"),
        HumanPlayerCone("Cone Signal"),
        Idle("Idle Light");

       // RAINBOW("Rainbow"),
       // PURPLEFLASH("Purple Flash"),
       // YELLOWFLASH("Yellow Flash"),
       // GREENFLASH("Green Flash"),
       // REDFLASH("Red Flash");

        private String name;

        private LEDMode(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }
}
