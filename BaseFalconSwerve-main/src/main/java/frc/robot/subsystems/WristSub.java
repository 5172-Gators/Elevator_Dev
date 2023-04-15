// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class WristSub extends SubsystemBase {
  /** Creates a new ElevatorTest. */
  //private WPI_CANCoder m_CANCoder;
  private final TalonFX wristMotor;


  // final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

  private static final double k_openLoopRampRate = 0.1;
  private static final int k_currentLimit = Constants.Wrist.currentLimit; // Current limit for intake falcon 500

  private double m_encoder = 0;
  private double m_goalPosition;

  public WristSub() {

    // initialize motors
    // the right motor will spin clockwise and the left motor will go counter
    // clockwise
    wristMotor = new TalonFX(Constants.Wrist.wristMotorID);
   //m_CANCoder = new WPI_CANCoder(Constants.Wrist.kWristCancoderID, "rio");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.openloopRamp = k_openLoopRampRate;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

    wristMotor.configAllSettings(config);
    wristMotor.enableVoltageCompensation(true);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setInverted(TalonFXInvertType.CounterClockwise);
    //wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // elevatorMotorOne.setSelectedSensorPosition(0); // zero the encoder
    //wristMotor.configRemoteFeedbackFilter(m_CANCoder, 0);

    // wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,
    //     Constants.Wrist.kPIDLoopIdx,
    //     Constants.Wrist.kTimeoutMs);
    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    
        Constants.Wrist.kPIDLoopIdx,
        Constants.Wrist.kTimeoutMs);

    wristMotor.setSensorPhase(Constants.Wrist.kSensorPhase);

    wristMotor.configAllowableClosedloopError(0, Constants.Wrist.kWristDeadband, Constants.Wrist.kTimeoutMs);
    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    // elevatorMotorOne.config_kF(Constants.Elevator.kPIDLoopIdx,
    // Constants.Elevator. kGains.kF, Constants.Elevator.kTimeoutMs);
    wristMotor.config_kP(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKP, Constants.Wrist.kTimeoutMs);
    wristMotor.config_kI(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKI, Constants.Wrist.kTimeoutMs);
    wristMotor.config_kD(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKD, Constants.Wrist.kTimeoutMs);
    wristMotor.config_kF(Constants.Wrist.kPIDLoopIdx, Constants.Wrist.wristKF, Constants.Wrist.kTimeoutMs);

    wristMotor.setSelectedSensorPosition(0);
    m_encoder = wristMotor.getSelectedSensorPosition(); // * 1.0 / 360.0 * 2.0 * Math.PI * 1.5;

  }





  public void setPosition(double goalPosition) {
    // if (goalPosition > Constants.Wrist.lowerLimit) {
    //   goalPosition = Constants.Wrist.lowerLimit;
    // } else if (goalPosition < Constants.Wrist.upperLimit) {
    //   goalPosition = Constants.Wrist.upperLimit;
    // }
    m_goalPosition = goalPosition;
  }

  public void joystickPosition(double joystickPosition) {
    m_goalPosition = m_goalPosition + joystickPosition;
  }

  public double WristPosition() {
    return wristMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_encoder = wristMotor.getSelectedSensorPosition();// * (1.0 / 360.0 * 2.0 * Math.PI * 1.5);

    SmartDashboard.putNumber("Wrist Position", m_encoder);
    SmartDashboard.putNumber("Wrist Goal Position", m_goalPosition);
    SmartDashboard.putNumber("Wrist Motor Output", wristMotor.getMotorOutputPercent());
    // elevatorMotorOne.set(ControlMode.Position,
    // m_controller.calculate(m_encoder));
    wristMotor.set(TalonFXControlMode.Position, m_goalPosition);

  }

  public boolean atSetpoint() {
    if (WristPosition() < m_goalPosition + Constants.Wrist.kWristDeadband
        || WristPosition() > m_goalPosition + Constants.Wrist.kWristDeadband) {
      return true;
    } else {
      return false;
    }
  }
}
