package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
//import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;

public class TeleopIntake extends CommandBase {
    private IntakeSub s_Intake;
    private double moveVal;
    private boolean m_in;
    private boolean m_out;

    public TeleopIntake(IntakeSub s_Intake, double moveVal) {
        this.s_Intake = s_Intake;
        this.moveVal = moveVal;
        addRequirements(s_Intake);
    }

    @Override
    public void execute() {

    //   if (m_in) {
    //         s_Intake.setMotor(Constants.Intake.coneIntakeSpeed);
    //     } else if (m_out) {
    //         s_Intake.setMotor(Constants.Intake.cubeIntakeSpeed);
    //     } else {
    //         s_Intake.setMotor(0);
    //     } 
    

    // public void end(boolean Interrupted){
     s_Intake.setMotor(moveVal);
    }

    @Override
    public boolean isFinished() {

        // s_Intake.setMotor(0);
        return true;
    }

}
