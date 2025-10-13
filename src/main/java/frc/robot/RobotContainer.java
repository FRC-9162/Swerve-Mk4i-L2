package frc.robot;
import frc.robot.Constants.Controle;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Inicializa subsistemas
  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final SendableChooser<Command> autoChooser;
  private CommandXboxController controleXbox = new CommandXboxController(Controle.xboxControle);

  public RobotContainer() {

    // Define o comando padrão como controle manual da Swerve
    swerve.setDefaultCommand(swerve.driveCommandAlinharComJoystick(
      () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
      () -> controleXbox.getRightX(),
      () -> controleXbox.getRightY(),
      () -> controleXbox.rightBumper().getAsBoolean()));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }
  private void configureBindings() {
    if(!Robot.isReal()){
      controleXbox.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    controleXbox.a().whileTrue(swerve.driveAlign45(
    () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
    () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND)));

    controleXbox.b().whileTrue(swerve.driveReefAlign(
    () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
    () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
    () -> controleXbox.rightBumper().getAsBoolean()
    ));
    controleXbox.y().onTrue(Commands.runOnce(
      () -> swerve.setTranslation(new Translation2d(1,4))
    ));


  }

  public void periodic() {
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void autoInit() {
    setMotorBrake(true);
  }

  public void teleOpinit() {
    
  }

  public void teleOP() {
    
  }
}
