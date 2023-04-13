// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
public class ElevatorSetPosition extends CommandBase {
  /** Creates a new TeleopElevatorTest. */
  private Elevator s_ElevatorTest;

  public ElevatorSetPosition(Elevator s_ElevatorTest) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ElevatorTest = s_ElevatorTest;

    addRequirements(s_ElevatorTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
      
      s_ElevatorTest.setPosition(30000); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(s_ElevatorTest.getElevatorPosition() < 30000 + 5000  && s_ElevatorTest.getElevatorPosition() > 30000 - 5000)
    {
      return true;
    } else
    {
      return false; 
    }
    
  }
}



