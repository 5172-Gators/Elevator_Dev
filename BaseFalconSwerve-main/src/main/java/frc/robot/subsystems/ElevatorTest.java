// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.Position;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorTest extends SubsystemBase {
  /** Creates a new ElevatorTest. */
  private static double kDt = 0.02;
  private final TalonFX elevatorMotorOne;
  private final TalonFX elevatorMotorTwo;
  final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

  private static final double k_openLoopRampRate = 0.1;
  private static final int k_currentLimit = Constants.Elevator.currentLimit; // Current limit for intake falcon 500

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(1.3, 0.0, 0.7, m_constraints, kDt);
  private double m_encoder = 0;

  public ElevatorTest() {

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
    elevatorMotorOne.setInverted(TalonFXInvertType.Clockwise);
    elevatorMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elevatorMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // elevatorMotorOne.setSelectedSensorPosition(0); // zero the encoder

    elevatorMotorTwo.configAllSettings(config);
    elevatorMotorTwo.enableVoltageCompensation(true);
    elevatorMotorTwo.setNeutralMode(NeutralMode.Brake);
    elevatorMotorTwo.setInverted(TalonFXInvertType.Clockwise);
    elevatorMotorTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_encoder = elevatorMotorOne.getSelectedSensorPosition() * 1.0 / 360.0 * 2.0 * Math.PI * 1.5;

  }

  public void setPosition(double position) {
    m_controller.setGoal(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_encoder = elevatorMotorOne.getSelectedSensorPosition() * (1.0 / 360.0 * 2.0 * Math.PI * 1.5);

    SmartDashboard.putNumber("Elevator Position", m_encoder);
    SmartDashboard.putNumber("Elevator Goal Position", m_controller.calculate(m_encoder));
    elevatorMotorOne.set(ControlMode.Position, m_controller.calculate(m_encoder));
  }
}
