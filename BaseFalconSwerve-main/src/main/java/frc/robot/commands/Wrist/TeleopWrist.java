// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSub;
public class TeleopWrist extends CommandBase {
  /** Creates a new TeleopElevatorTest. */
  private WristSub s_Wrist;
  private DoubleSupplier s_GoalPosition;

  public TeleopWrist(WristSub s_Wrist, DoubleSupplier GoalPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Wrist = s_Wrist;
    this.s_GoalPosition = GoalPosition;

    addRequirements(s_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
      s_Wrist.joystickPosition(s_GoalPosition.getAsDouble()); 
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