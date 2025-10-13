package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.concurrent.Flow.Publisher;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConfigs;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/** 
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class SwerveSubsystem extends SubsystemBase {
    // Objeto global da SwerveDrive (Classe YAGSL)
    SwerveDrive swerveDrive;
    StructPublisher<Pose2d> poseVisionPublisher = NetworkTableInstance.getDefault().getStructTopic("PoseVision", Pose2d.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();

    Translation2d blueReefCenter = new Translation2d(4.5, 4);
    Translation2d redReefCenter = new Translation2d(13, 4);
    // Método construtor da classe
    public SwerveSubsystem(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Acessa os arquivos do diretório .JSON
        try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Dimensoes.MAX_SPEED);
        } catch (Exception e) {
          throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(Constants.SwerveConfigs.headingCorrection);
        swerveDrive.angularVelocityCorrection =  SwerveConfigs.usarCorrecaoDesvioVelocidadeAngular;
        swerveDrive.angularVelocityCoefficient = SwerveConfigs.coeficienteCorecaoAngVel;
        
        setupPathPlanner();
        swerveDrive.stopOdometryThread();
    }
    
    @Override
    public void periodic() {
      // Dentro da função periódica atualizamos nossa odometria
      swerveDrive.updateOdometry();
      LimelightHelpers.SetRobotOrientation("limelight-front",getHeading().getDegrees(),0,0,0,0,0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
   
      boolean doRejectUpdate = false;
      posePublisher.set(getPose());

  // se nossa velocidade angular for maior que 360 graus por segundo, ignore atualizações de visão
      if(Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 180)
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
            poseVisionPublisher.set(mt2.pose);
      }
    
  }

      public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5,0,0),
              // Translation PID constants/
              new PIDConstants(5,0,0)
              // Rotation PID constants 
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  //Movimenta o robô com o joystick esquerdo, e mira o robo no ângulo no qual o joystick está apontando
  public Command driveCommandAlinharComJoystick(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY, BooleanSupplier boostSupplier)
  {
    return run(() -> {
      var alliance = DriverStation.getAlliance();
      double xInput = translationX.getAsDouble(); 
      double yInput = translationY.getAsDouble();
      double xHeading = headingX.getAsDouble();
      double yHeading = headingY.getAsDouble();

      if (alliance.isPresent()){
        if(alliance.get() == Alliance.Blue){
          xInput = -xInput;
          yInput = -yInput;
          xHeading = -headingX.getAsDouble();
          yHeading = -headingY.getAsDouble();
        }
      }
      Translation2d inputs = new Translation2d(xInput , yInput);
      if(boostSupplier.getAsBoolean()){
        inputs = SwerveMath.scaleTranslation(inputs, 1);
      }else{
        inputs = SwerveMath.scaleTranslation(inputs, 0.6);
      }

      // Faz o robô se mover
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(inputs.getX(), inputs.getY(),
                                                                      xHeading,
                                                                      yHeading,
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  //Movimenta o robô com o joystick esquerdo, e gira o robô na intensidade na qual o joystick direito está para o lado
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      // Faz o robô se mover
      swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumChassisVelocity(),
                                          yInput * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public Command driveAlign45(DoubleSupplier translationX, DoubleSupplier translationY)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      double omega = swerveDrive.swerveController.headingCalculate(this.getHeading().getRadians(), Rotation2d.fromDegrees(45).getRadians());
      // Faz o robô se mover
      swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumChassisVelocity(),
                                          yInput * swerveDrive.getMaximumChassisVelocity()),
                        omega,
                        true,
                        false);                 
    });
  }

  public Command driveReefAlign(DoubleSupplier translationX, DoubleSupplier translationY, BooleanSupplier boostSupplier)
  {
    return run(() -> {
      var alliance = DriverStation.getAlliance();
      double xInput = translationX.getAsDouble(); 
      double yInput = translationY.getAsDouble();

      if (alliance.isPresent()){
        if(alliance.get() == Alliance.Blue){
          xInput = -xInput;
          yInput = -yInput;
        }
      }
      Translation2d inputs = new Translation2d(xInput , yInput);
      if(boostSupplier.getAsBoolean()){
        inputs = SwerveMath.scaleTranslation(inputs, 1);
      }else{
        inputs = SwerveMath.scaleTranslation(inputs, 0.6);
      }
      Rotation2d targetAngle;
      if(getPose().getX() > 8){
        targetAngle = getPose().getTranslation().minus(redReefCenter).getAngle();
        
      }else{
        targetAngle = getPose().getTranslation().minus(blueReefCenter).getAngle();
      };
      targetAngle = targetAngle.plus(Rotation2d.k180deg);
      double omega = swerveDrive.swerveController.headingCalculate(this.getHeading().getRadians(), targetAngle.getRadians());

      swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumChassisVelocity(),
                                          yInput * swerveDrive.getMaximumChassisVelocity()),
                        omega,
                        true,
                        false);                 
    });
  }

  public void setTranslation(Translation2d translation2d){
    Pose2d pose = new Pose2d(translation2d, getHeading());
    resetOdometry(pose);
  }

    public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

    // Função drive que chamamos em nossa classe de comando Teleoperado
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) 
    {
      swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    // Função para obter a velocidade desejada a partir dos inputs do gamepad
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, 
      getHeading().getRadians());
    }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) 
  {
    return new ChassisSpeeds(xInput, yInput, 0);
  }

  // Função que retorna a posição do robô (translação e ângulo), (Usado no autônomo)
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  
  // Retorna a velocidade relativa ao campo
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  // Retorna a configuração do swerve
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Retorna o objeto de controle, o qual é usado para acessar as velocidades máximas por exemplo
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  // Ângulo atual do robô
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  // Reseta a odometria para uma posição indicada (Usado no autônomo)
  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  // Seta a velocidade do chassi (Usado no autônomo)
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

    public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);

  }

  public void stopRobot(){
    drive(new Translation2d(0.0 , 0.0), 0, true);
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }
}