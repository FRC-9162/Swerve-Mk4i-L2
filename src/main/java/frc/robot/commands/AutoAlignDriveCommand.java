package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.Controle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutoAlignDriveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final Limelight limelight;
  private final CommandXboxController controle;

  public AutoAlignDriveCommand(SwerveSubsystem swerve, Limelight limelight, CommandXboxController controle) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.controle = controle;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double tx = limelight.getTX(); 
    double ta = limelight.getTA(); 
    boolean temTag = limelight.getTV() == 1.0;

    double vx = MathUtil.applyDeadband(controle.getLeftY(), Controle.DEADBAND);
    double vy = MathUtil.applyDeadband(controle.getLeftX(), Controle.DEADBAND);
    double omega = MathUtil.applyDeadband(controle.getRightX(), Controle.DEADBAND);

   
    if (temTag && ta > 3.0) {
      double kP = 0.06;
      double offset = 2.0;
      double erro = tx - offset;
      vy = MathUtil.clamp(kP * erro, -0.5, 0.5); 
      vx = -0.4;
    }

    swerve.drive(new Translation2d(vx, vy), omega, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}