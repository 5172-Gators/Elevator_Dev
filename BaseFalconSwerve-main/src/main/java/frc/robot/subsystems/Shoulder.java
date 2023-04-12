// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;





import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.Constants.Wrist.PIDFFmode;
//import frc.robot.Constants.Wrist.PIDFFmode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
public class Shoulder extends SubsystemBase {
    private final TalonFX ShoulderMotor;

    /**
     * Declares the relative and absolute encoders for the wrist. The absolute
     * encoder
     * is the through bore encoder and the relative encoder is the encoder that is
     * built into the
     * spark max motor.
     */
    //private final RelativeEncoder relativeEncoder;
    private final CANCoder ShoulderAbsoluteEncoder;

    private double currentPosition;
    private double m_goalPosition;
    private final PIDController pidController;
    private ArmFeedforward feedForward;
    private static final double k_openLoopRampRate = 0.1;
    //private Encoder relativeEncoder;
    private double relativeEncoderPos;
    /**
     * Intake
     */
    public Shoulder() {

        ShoulderMotor = new TalonFX(Constants.Shoulder.ShoulderMotorID);//new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
        ShoulderAbsoluteEncoder = new CANCoder(Constants.Shoulder.EncoderID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, Constants.Shoulder.currentLimit, 0, 0);

        
        ShoulderMotor.configAllSettings(config);
        ShoulderMotor.enableVoltageCompensation(true);
        ShoulderMotor.setNeutralMode(NeutralMode.Brake);
        ShoulderMotor.setInverted(TalonFXInvertType.Clockwise);
        ShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        //absoluteEncoder = //wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        //absoluteEncoder.setInverted(true);

        pidController = new PIDController(Constants.Shoulder.unweightedP, Constants.Shoulder.unweightedI,
                Constants.Shoulder.unweightedD);
        pidController.enableContinuousInput(0, Math.PI * 2);
        pidController.setTolerance(.25);

        relativeEncoderPos = (ShoulderMotor.getSelectedSensorPosition()*Constants.Shoulder.motorGearRatio * 2 * Math.PI);
        
        
        ShoulderAbsoluteEncoder.configMagnetOffset(Constants.Shoulder.absoluteEncoderOffset);
      
        feedForward = new ArmFeedforward(Constants.Shoulder.unweightedS, Constants.Shoulder.unweightedG,
                Constants.Shoulder.unweightedV,
                Constants.Shoulder.unweightedA);

        //relativeEncoder.conf setPosition(wristAbsoluteEncoder.getPosition());

        setPosition(Position.STANDBY.getShoulder());

        
    }

    public void resetRelativeEncoder() {
      ShoulderMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shoulder Relative Encoder",
        // relativeEncoder.getPosition());
        SmartDashboard.putBoolean("Shoulder at setpoint", atSetpoint());
        SmartDashboard.putNumber("Shoulder Absolute Position", ShoulderAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder Goal position", currentPosition);

        double pidMotorSpeed = pidController.calculate(ShoulderAbsoluteEncoder.getPosition(), currentPosition)
                + feedForward.calculate(currentPosition, 0);
        SmartDashboard.putNumber("Motor power shoulder", pidMotorSpeed);
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
      ShoulderMotor.setSelectedSensorPosition(0);
    }

    public double getEncoderPosition() {
        return ShoulderAbsoluteEncoder.getPosition();
    }

    public void setMotor(double voltage) {
      ShoulderMotor.set(ControlMode.PercentOutput, voltage/12);
    }

    public Command setPositionCMD(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double goalPosition) { // should we find a way to add the lowest position into this?
        if (goalPosition < Constants.Shoulder.upperLimit){
          goalPosition = Constants.Shoulder.upperLimit;
        }
        m_goalPosition = goalPosition;
      }

    public double getPosition() {
        return currentPosition;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

}