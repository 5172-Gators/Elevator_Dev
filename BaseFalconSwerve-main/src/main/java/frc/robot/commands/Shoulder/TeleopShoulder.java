// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSub;

public class TeleopShoulder extends CommandBase {
  /** Creates a new TeleopElevatorTest. */
  private ShoulderSub s_Shoulder;
  private DoubleSupplier s_GoalPosition;

  public TeleopShoulder(ShoulderSub s_Shoulder, DoubleSupplier GoalPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shoulder = s_Shoulder;
    this.s_GoalPosition = GoalPosition;

    addRequirements(s_Shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
      
    s_Shoulder.joystickPosition(s_GoalPosition.getAsDouble()); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}



