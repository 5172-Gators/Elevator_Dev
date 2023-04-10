package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.Position;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Gains;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {

    // private final CANSparkMax elevatorMotorLeft; // making the left the lead
    // motor
    // private final CANSparkMax elevatorMotorRight; // the right motor is the
    // follower
    private final TalonFX elevatorMotorOne;
    private final TalonFX elevatorMotorTwo;

    private static final double k_openLoopRampRate = 0.1;
    private static final int k_currentLimit = Constants.Elevator.currentLimit; // Current limit for intake falcon 500

    private PIDController pidController;

    private double currentPosition;

    /**
     * Initialize Elevator motor and the built in encoder. There are no cancoders on
     * the elevator
     */
    public Elevator() {
        // initialize motors
   
        elevatorMotorOne = new TalonFX(RobotMap.ELEVATOR_MASTER_MOTOR);
        elevatorMotorTwo = new TalonFX(RobotMap.ELEVATOR_SLAVE_MOTOR);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.openloopRamp = k_openLoopRampRate;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, k_currentLimit, 0, 0);

        elevatorMotorOne.configAllSettings(config);
        elevatorMotorOne.enableVoltageCompensation(true);
        elevatorMotorOne.setNeutralMode(NeutralMode.Brake);
        elevatorMotorOne.setInverted(TalonFXInvertType.Clockwise);
        elevatorMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        elevatorMotorTwo.configAllSettings(config);
        elevatorMotorTwo.enableVoltageCompensation(true);
        elevatorMotorTwo.setNeutralMode(NeutralMode.Brake);
        elevatorMotorTwo.setInverted(TalonFXInvertType.Clockwise);
        elevatorMotorTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


        // initialize pidContoller
        pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI,
                Constants.Elevator.elevatorKD);
        pidController.setSetpoint(0);
        pidController.setTolerance(.25);

        setPosition(Position.STANDBY.getElev());
    }

    public void resetEncoder() {
        elevatorMotorOne.getEncoder().setPosition(0);
        elevatorMotorTwo.getEncoder().setPosition(0);
    }

    public Command setPositionCMD(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double position) {
        if (position > 35) {
            position = 35;
        } else if (position < 0.1) {
            position = 0.1;
        }
        currentPosition = position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public void move(double voltage) {
        elevatorMotorLeft.setVoltage(voltage);
    }

    public boolean reachedSetpoint(double distance) {
        return pidController.getPositionTolerance() >= Math.abs(currentPosition - distance);
    }

    public double getEncoderPosition() {
        return (elevatorMotorLeft.getEncoder().getPosition() + elevatorMotorRight.getEncoder().getPosition()) / 2;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Position", getEncoderPosition());
        SmartDashboard.putNumber("Elevator Goal Position", currentPosition);

        move(
                MathUtil.clamp(
                        pidController.calculate(getEncoderPosition(), currentPosition),
                        -Constants.Elevator.maxMotorVoltage,
                        Constants.Elevator.maxMotorVoltage));
    }
}