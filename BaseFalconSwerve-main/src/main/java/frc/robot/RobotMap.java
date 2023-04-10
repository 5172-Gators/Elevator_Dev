// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The RobotMap is a mapping from the physical wiring of the robot to variable names. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    // Power Distribution Hub CAN ID
    public static final int PDH = 0;

    // Pneumatic Hub CAN ID
    public static final int PNEUMATIC_HUB = 1;

    // Drive Subsystem
    public static final int FRONT_LEFT_DRIVE_MOTOR_PDH = 1;
    public static final int FRONT_LEFT_STEER_MOTOR_PDH = 2;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PDH = 3;
    public static final int FRONT_RIGHT_STEER_MOTOR_PDH = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR_PDH = 5;
    public static final int BACK_LEFT_STEER_MOTOR_PDH = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR_PDH = 7;
    public static final int BACK_RIGHT_STEER_MOTOR_PDH = 8;
    // Intake Subsystem
    public static final int RIGHT_INTAKE_MOTOR_CAN = 53;
    public static final int LEFT_INTAKE_MOTOR_CAN = 22;
    public static final int INTAKE_SOL = 13;
      
    // Arm Subsystem
    public static final int ELEVATOR_MASTER_MOTOR = 50;
    public static final int ELEVATOR_SLAVE_MOTOR = 23;
    public static final int JOINT_ONE_CANCODER = 16;


    public static final int JOINT_TWO_MASTER_MOTOR = 42;
    public static final int JOINT_TWO_SLAVE_MOTOR = 41;
    public static final int JOINT_TWO_CANCODER = 14;

    public static final int JOINT_THREE_MOTOR = 52;
    public static final int JOINT_THREE_CANCODER = 15;

    public static final int CLAW_ROTATING_MOTOR = 1;
    
    // Claw Subsystem
    public static final int PRESSURE_SELECT_SOL = 0;
    public static final int CLAW_SOL = 14;



   

    // LED Subsystem
    public static final int CANIFIER_CAN = 21;

	
    }
