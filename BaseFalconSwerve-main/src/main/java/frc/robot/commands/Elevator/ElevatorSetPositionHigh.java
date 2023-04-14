// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSub;
public class ElevatorSetPositionHigh extends CommandBase {
  /** Creates a new TeleopElevatorTest. */
  private ElevatorSub s_Elevator;

  public ElevatorSetPositionHigh(ElevatorSub s_Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Elevator = s_Elevator;

    addRequirements(s_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
      
      s_Elevator.setPosition(55000); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(s_Elevator.ElevatorPosition() < 55000 + 5000  && s_Elevator.ElevatorPosition() > 55000 - 5000)
    {
      return true;
    } else
    {
      return false; 
    }
    
  }
}



