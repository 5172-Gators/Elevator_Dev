// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.ShoulderSub;
import frc.robot.subsystems.WristSub;

public class SetAllPositions extends CommandBase {
  private WristSub s_Wrist;
  private ElevatorSub s_Elevator;
  private ShoulderSub s_Shoulder;

  private Position k_position;
  private Timer timer;
  private Supplier<GamePiece> k_gamePiece;

  /** Creates a new SetAllPositions. */
  public SetAllPositions(WristSub s_Wrist, ElevatorSub s_Elevator, ShoulderSub s_Shoulder, Position k_position,
      Supplier<GamePiece> k_gamePiece) {
    this.s_Wrist = s_Wrist;
    this.s_Elevator = s_Elevator;
    this.s_Shoulder = s_Shoulder;
    this.k_position = k_position;
    this.k_gamePiece = k_gamePiece;
    this.timer = new Timer();

    addRequirements(s_Wrist, s_Elevator, s_Shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (k_position) {
      case HIGH:
        if (k_gamePiece.get() == GamePiece.CONE) {
          s_Wrist.setPosition(Position.CONEHIGH.getWrist());
          s_Elevator.setPosition(Position.CONEHIGH.getElev());
          s_Shoulder.setPosition(Position.CONEHIGH.getShoulder());

        } else if (k_gamePiece.get() == GamePiece.CUBE) {
          s_Wrist.setPosition(Position.CUBEHIGH.getWrist());
          s_Elevator.setPosition(Position.CUBEHIGH.getElev());
          s_Shoulder.setPosition(Position.CUBEHIGH.getShoulder());
        }
        break;

      case MID:
        if (k_gamePiece.get() == GamePiece.CONE) {
          s_Wrist.setPosition(Position.CONEMID.getWrist());
          s_Elevator.setPosition(Position.CONEMID.getElev());
          s_Shoulder.setPosition(Position.CONEMID.getShoulder());

        } else if (k_gamePiece.get() == GamePiece.CUBE) {
          s_Wrist.setPosition(Position.CUBEMID.getWrist());
          s_Elevator.setPosition(Position.CUBEMID.getElev());
          s_Shoulder.setPosition(Position.CUBEMID.getShoulder());
        }
        break;

      case LOW:
        if (k_gamePiece.get() == GamePiece.CONE) {
          s_Wrist.setPosition(Position.CONELOW.getWrist());
          s_Elevator.setPosition(Position.CONELOW.getElev());
          s_Shoulder.setPosition(Position.CONELOW.getShoulder());

        } else if (k_gamePiece.get() == GamePiece.CUBE) {
          s_Wrist.setPosition(Position.CUBELOW.getWrist());
          s_Elevator.setPosition(Position.CUBELOW.getElev());
          s_Shoulder.setPosition(Position.CONELOW.getShoulder());
        }
        break;

      default:
        s_Wrist.setPosition(k_position.getWrist());
        s_Elevator.setPosition(k_position.getElev());
        s_Shoulder.setPosition(k_position.getShoulder());
        break;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    timer.stop();
    timer.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Wrist.atSetpoint() && s_Elevator.atSetpoint() && s_Shoulder.atSetpoint()) || timer.get() > .75;
  }
}
