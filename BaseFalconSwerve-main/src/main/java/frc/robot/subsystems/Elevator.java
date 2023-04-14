// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.ControlMode;

//import java.util.function.DoubleSupplier;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import frc.robot.Constants.Position;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class Elevator extends SubsystemBase {
  /** Creates a new ElevatorTest. */
  //private static double kDt = 0.02;
  private final TalonFX elevatorMotorOne;
  private final TalonFX elevatorMotorTwo;
  // final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

  private static final double k_openLoopRampRate = 0.1;
  private static final int k_currentLimit = Constants.Elevator.currentLimit; // Current limit for intake falcon 500

  // private final TrapezoidProfile.Constraints m_constraints = new
  // TrapezoidProfile.Constraints(1.75, 0.75);
  // private final ProfiledPIDController m_controller = new
  // ProfiledPIDController(1.3, 0.0, 0.7, m_constraints, kDt);
  private double m_encoder = 0;
  private double m_goalPosition;

  public Elevator() {

    // initialize motors
    // the right motor will spin clockwise and the left motor will go counter
    // clockwise
    elevatorMotorOne = new TalonFX(Constants.Elevator.motorOneId);
    elevatorMotorTwo = new TalonFX(Constants.Elevator.motorTwoId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.openloopRamp = k_openLoopRampRate;
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

    elevatorMotorOne.configAllSettings(config);
    elevatorMotorOne.enableVoltageCompensation(true);
    elevatorMotorOne.setNeutralMode(NeutralMode.Brake);
    elevatorMotorOne.setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elevatorMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // elevatorMotorOne.setSelectedSensorPosition(0); // zero the encoder

    elevatorMotorTwo.configAllSettings(config);
    elevatorMotorTwo.enableVoltageCompensation(true);
    elevatorMotorTwo.setNeutralMode(NeutralMode.Brake);
    elevatorMotorTwo.setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotorTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);    
    elevatorMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    
    elevatorMotorOne.setSensorPhase(Constants.Elevator.kSensorPhase);
    elevatorMotorTwo.setSensorPhase(Constants.Elevator.kSensorPhase);


    elevatorMotorTwo.follow(elevatorMotorOne);
    /* Config the sensor used for Primary PID and sensor direction */
    elevatorMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Elevator.kPIDLoopIdx,
        Constants.Elevator.kTimeoutMs);
    // elevatorMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    //     Constants.Elevator.kPIDLoopIdx,
    //     Constants.Elevator.kTimeoutMs);

    elevatorMotorOne.configAllowableClosedloopError(0, Constants.Elevator.kmaxAllowableError, Constants.Elevator.kTimeoutMs);
    //elevatorMotorTwo.configAllowableClosedloopError(0, Constants.Elevator.kmaxAllowableError, Constants.Elevator.kTimeoutMs);
    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    // elevatorMotorOne.config_kF(Constants.Elevator.kPIDLoopIdx,
    // Constants.Elevator. kGains.kF, Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kP(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKP,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kI(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKI,
        Constants.Elevator.kTimeoutMs);
    elevatorMotorOne.config_kD(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKD,
        Constants.Elevator.kTimeoutMs);

    // elevatorMotorTwo.config_kP(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKP,
    //     Constants.Elevator.kTimeoutMs);
    // elevatorMotorTwo.config_kI(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKI,
    //     Constants.Elevator.kTimeoutMs);
    // elevatorMotorTwo.config_kD(Constants.Elevator.kPIDLoopIdx, Constants.Elevator.elevatorKD,
    //     Constants.Elevator.kTimeoutMs);

    resetEncoder();
    m_encoder = elevatorMotorOne.getSelectedSensorPosition();
    setPosition(Position.STANDBY.getElev());
  }

  public void setPosition(double goalPosition) {
    if (goalPosition < Constants.Elevator.maxExtension) { // limit goalPosition to sane values
      goalPosition = Constants.Elevator.maxExtension;
    } else if (goalPosition > Constants.Elevator.minExtension) {
      goalPosition = Constants.Elevator.minExtension;
    }
    this.m_goalPosition = goalPosition;
    SmartDashboard.putNumber("setPosition: Elevator Goal Position", m_goalPosition);
  }

  public void resetEncoder() {
    elevatorMotorOne.setSelectedSensorPosition(0);
    elevatorMotorTwo.setSelectedSensorPosition(0);
  }

  public void joystickPosition(double joystickPosition) {
    m_goalPosition = m_goalPosition + joystickPosition; // add Joystick modifier if operator has to tweak positions
    SmartDashboard.putNumber("Elevator Goal Position w/ Joystick", m_goalPosition);
  }

  public double getElevatorPosition() {
    return (elevatorMotorOne.getSelectedSensorPosition() + elevatorMotorTwo.getSelectedSensorPosition()) / 2; // average
                                                                                                              // the two
                                                                                                              // encoders
  }

  public boolean atSetpoint() {
    if (elevatorMotorOne.getSelectedSensorPosition() == m_goalPosition) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_encoder = elevatorMotorOne.getSelectedSensorPosition();// * (1.0 / 360.0 * 2.0 * Math.PI * 1.5);

    SmartDashboard.putNumber("Elevator Position", m_encoder);
    SmartDashboard.putNumber("Elevator Goal Position", m_goalPosition);
    // elevatorMotorOne.set(ControlMode.Position,
    // m_controller.calculate(m_encoder));
    elevatorMotorOne.set(TalonFXControlMode.Position, m_goalPosition);
    //elevatorMotorTwo.set(TalonFXControlMode.Position, m_goalPosition);
 }

}
