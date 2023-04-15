package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
//import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;

public class intakeStop extends CommandBase {
    private IntakeSub s_Intake;
    private double moveVal;

    public intakeStop(IntakeSub s_Intake) {
        this.s_Intake = s_Intake;
        this.moveVal = moveVal;

        addRequirements(s_Intake);
    }

    @Override
    public void execute() {
        s_Intake.setMotor(0);
        
    }

    public void end(boolean Interrupted){
       // s_Intake.stopIntake();
    }

    @Override
    public boolean isFinished() {

        //s_Intake.setMotor(0);
      return true;
    }
    
}