package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

/** Main robot class */
public class Robot extends TimedRobot {

    private Command m_autoCommand;
    private Thread visionThread;
    private volatile boolean running = true;

    // Shuffleboard entries
    private GenericEntry xOffsetEntry;
    private GenericEntry yOffsetEntry;
    private GenericEntry sizeEntry;
    private GenericEntry showCrosshairEntry;

    @Override
    public void robotInit() {
        new RobotContainer();

        // Create a Shuffleboard tab for camera controls
        ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

        xOffsetEntry = cameraTab.add("Crosshair X Offset", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -160, "max", 160))
            .getEntry();

        yOffsetEntry = cameraTab.add("Crosshair Y Offset", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -120, "max", 120))
            .getEntry();

        sizeEntry = cameraTab.add("Crosshair Size", 20.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 5, "max", 100))
            .getEntry();

        showCrosshairEntry = cameraTab.add("Show Crosshair", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        // Start the vision thread
        visionThread = new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture("USB Camera 0", 0);
            camera.setResolution(320, 240);
            camera.setBrightness(50);
            camera.setFPS(20);

            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Crosshair Camera", 320, 240);

            Mat source = new Mat();
            Mat output = new Mat();

            while (running) {
                if (cvSink.grabFrame(source) == 0) {
                    outputStream.notifyError(cvSink.getError());
                    continue;
                }

                source.copyTo(output);

                boolean showCrosshair = showCrosshairEntry.getBoolean(true);
                if (showCrosshair) {
                    drawCrosshair(output);
                }

                outputStream.putFrame(output);
            }

            source.release();
            output.release();
        });

        visionThread.setDaemon(true);
        visionThread.start();
    }

    private void drawCrosshair(Mat frame) {
        int width = frame.width();
        int height = frame.height();

        double xOffset = xOffsetEntry.getDouble(0.0);
        double yOffset = yOffsetEntry.getDouble(0.0);
        double size = sizeEntry.getDouble(20.0);

        int centerX = width / 2 + (int) xOffset;
        int centerY = height / 2 + (int) yOffset;

        Scalar color = new Scalar(0, 255, 0); // green crosshair

        // Draw horizontal and vertical lines
        Imgproc.line(frame, new Point(centerX - size, centerY), new Point(centerX + size, centerY), color, 2);
        Imgproc.line(frame, new Point(centerX, centerY - size), new Point(centerX, centerY + size), color, 2);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Shuffleboard.update();
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Time", Timer.getMatchTime());
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {}
    @Override
    public void disabledPeriodic() {}
    @Override
    public void autonomousPeriodic() {}
    @Override
    public void teleopPeriodic() {}
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    @Override
    public void testPeriodic() {}

    @Override
    public void disabledExit() {
        running = false;
        try {
            visionThread.join(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}