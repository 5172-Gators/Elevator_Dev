package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
//import frc.robot.Constants.Wrist.PIDFFmode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.ctre.phoenix.sensors.CANCoder;
/**
 * The wrist subsystem will be used to set up the motors and encoders for the
 * wrist.
 */
public class Wrist extends SubsystemBase {
    private final TalonFX wristMotor;

    /**
     * Declares the relative and absolute encoders for the wrist. The absolute
     * encoder
     * is the through bore encoder and the relative encoder is the encoder that is
     * built into the
     * spark max motor.
     */
    //private final RelativeEncoder relativeEncoder;
    private final CANCoder wristAbsoluteEncoder;

    private double currentPosition;
    private final PIDController pidController;
    private ArmFeedforward feedForward;
    private static final double k_openLoopRampRate = 0.1;
    private Encoder relativeEncoder;
    private double relativeEncoderPos;
    /**
     * Intake
     */
    public Wrist() {

        wristMotor = new TalonFX(Constants.Wrist.wristMotorID);//new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
        wristAbsoluteEncoder = new CANCoder(Constants.Wrist.EncoderID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, Constants.Wrist.currentLimit, 0, 0);

        
        wristMotor.configAllSettings(config);
        wristMotor.enableVoltageCompensation(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(TalonFXInvertType.Clockwise);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        //absoluteEncoder = //wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        //absoluteEncoder.setInverted(true);

        pidController = new PIDController(Constants.Wrist.unweightedP, Constants.Wrist.unweightedI,
                Constants.Wrist.unweightedD);
        pidController.enableContinuousInput(0, Math.PI * 2);
        pidController.setTolerance(.25);

        relativeEncoderPos = wristMotor.getSelectedSensorPosition();
        
        wristAbsoluteEncoder.configMagnetOffset(Constants.Wrist.absoluteEncoderOffset);
        //relativeEncoderPos.configFeedbackCoefficient.sensorCoefficient(Constants.Wrist.motorGearRatio * 2 * Math.PI);

        feedForward = new ArmFeedforward(Constants.Wrist.unweightedS, Constants.Wrist.unweightedG,
                Constants.Wrist.unweightedV,
                Constants.Wrist.unweightedA);

        relativeEncoder.conf setPosition(wristAbsoluteEncoder.getPosition());

        setPosition(Position.STANDBY.getWrist());

        wristMotor.setSmartCurrentLimit(Constants.Wrist.currentLimit);
    }

    public void resetRelativeEncoder() {
        wristMotor.getEncoder().setPosition(0);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Wrist Relative Encoder",
        // relativeEncoder.getPosition());
        SmartDashboard.putBoolean("Wrist at setpoint", atSetpoint());
        SmartDashboard.putNumber("Wrist Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Goal position", currentPosition);

        double pidMotorSpeed = pidController.calculate(absoluteEncoder.getPosition(), currentPosition)
                + feedForward.calculate(currentPosition, 0);
        SmartDashboard.putNumber("Motor power wrist", pidMotorSpeed);
        setMotor(
                MathUtil.clamp(
                        (pidMotorSpeed), -Constants.Wrist.maxMotorVoltage,
                        Constants.Wrist.maxMotorVoltage));

    }

    public void setPIDFFMode(PIDFFmode mode) {
        pidController.setPID(mode.kP, mode.kI, mode.kD);
        feedForward = new ArmFeedforward(mode.kS, mode.kG, mode.kV, mode.kA);
        if(mode == PIDFFmode.WEIGHTED){
        SmartDashboard.putNumber("PIDFF", 1);
        } else{
            SmartDashboard.putNumber("PIDFF", 0);
        }
        
    }

    public void resetEncoder() {
        wristMotor.getEncoder().setPosition(0);

    }

    public double getEncoderPosition() {
        return absoluteEncoder.getPosition();
    }

    public void setMotor(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public Command setPositionCMD(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double position) {
        currentPosition = position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

}