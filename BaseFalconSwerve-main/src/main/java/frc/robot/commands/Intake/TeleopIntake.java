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

public class TeleopIntake extends CommandBase {
    private IntakeSub s_Intake;
    private DoubleSupplier moveVal;

    public TeleopIntake(IntakeSub s_Intake, DoubleSupplier moveVal) {
        this.s_Intake = s_Intake;
        this.moveVal = moveVal;

        addRequirements(s_Intake);
    }

    @Override
    public void execute() {
        double maxSpeed = RobotContainer.gamePiece == GamePiece.CONE ? Constants.Intake.coneIntakeSpeed
                : Constants.Intake.cubeIntakeSpeed;

        double power = MathUtil.clamp(
                ((moveVal.getAsDouble()) * maxSpeed + .5) * RobotContainer.gamePiece.getDirection(),
                -maxSpeed,
                maxSpeed);

        s_Intake.setMotor(power);

        // Check if a cone was intaked, if so switch PID on wrist.
        // if (power != 0 &&
        //         Math.abs(s_Intake.getVelocity()) < Constants.Intake.stoppedRPMThreshold
        //         && RobotContainer.gamePiece == GamePiece.CONE) {
        //     s_WristSub.setPIDFFMode(PIDFFmode.WEIGHTED);
        // }
    }
}
