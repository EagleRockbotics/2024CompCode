// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class Camera {
  PhotonCamera self;

  public double yOffset;
  public double xOffset;
  public double zOffset;

  public double pitchOffest;
  public double yawOffset;
  public double rollOffset;

  HashMap<Integer, Float> tags = new HashMap<>();

  public List<PhotonTrackedTarget> getVisibleTags() {
    var result = self.getLatestResult();
    return result.getTargets();
  }

  public void printVisibleTags() {
    for (PhotonTrackedTarget target : getVisibleTags()) {
      System.out.println(target.getFiducialId());
      System.out.println(target.getYaw());
      System.out.println(target.getPitch());
    }
  }

  public Camera(String nam) {
    self = new PhotonCamera(nam);
    pitchOffest = 0;
    yawOffset = 0;
    rollOffset = 0;

    xOffset = 0;
    yOffset = 0;
    zOffset = 0;

  }

  public Camera(String name, double pOffset, double yaOffset, double rOffset, double xoffset, double yoffset,
      double zoffset) {
    SmartDashboard.putString("Photonvision Camera Name", name);
    self = new PhotonCamera(name);
    pitchOffest = pOffset;
    yawOffset = yaOffset;
    rollOffset = rOffset;
    // side-side
    xOffset = xoffset;
    // forward-backward
    yOffset = yoffset;
    // up-down
    zOffset = zoffset;
  }
}

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
  Map<Integer, Double> tagOffset = new HashMap<>();
  Map<Integer, Pose2d> tagLocations = new HashMap<>();

  HashMap<Camera, List<PhotonTrackedTarget>> tags = new HashMap<>();

  List<Camera> cameras = new ArrayList();

  public VisionSubsystem() {
    try {
      tagOffset = mapper
          .readValue(Filesystem
              .getDeployDirectory()
              .toPath()
              .resolve(Constants.APRILTAG_JSON_NAME)
              .toFile(), new TypeReference<Map<Integer, Double>>() {
              });
      JsonNode camRoot = mapper.readTree(Filesystem
          .getDeployDirectory()
          .toPath()
          .resolve(Constants.CAMERA_JSON_NAME)
          .toFile());
      JsonNode tagPosRoot = mapper
          .readTree(Filesystem.getDeployDirectory().toPath().resolve(Constants.APRILTAG_POSITION_JSON_NAME).toFile());
      for (JsonNode node : tagPosRoot) {
        tagLocations.put(node.get("id").asInt(), new Pose2d(node.get("x").asDouble(), node.get("y").asDouble(),
            Rotation2d.fromDegrees(node.get("rotation").asDouble())));
      }
      for (JsonNode node : camRoot) {
        cameras.add(new Camera(node.get("name").asText(),
            node.get("pitch").asDouble(),
            node.get("yaw").asDouble(),
            node.get("roll").asDouble(),
            node.get("x").asDouble(),
            node.get("y").asDouble(),
            node.get("z").asDouble()));
      }

    } catch (Exception e) {
      e.printStackTrace();
    }

  }

  @Override
  public void periodic() {
    updateTagList();
    // for (Map.Entry<Integer, Transform2d> entry :
    // getDistances(Rotation2d.fromDegrees(0)).entrySet()) {
    // System.out.println("ID:" + entry.getKey().toString() + " Distances:" + new
    // Double(entry.getValue().getX()).toString() + " ," + new
    // Double(entry.getValue().getY()).toString());
    // }
  }

  void updateTagList() {
    for (Camera camera : cameras) {
      tags.put(camera, camera.getVisibleTags());
    }
  }

  public HashMap<Integer, Transform2d> getDistances(Rotation2d robotAngle) {
    HashMap<Integer, Transform2d> out = new HashMap<>();
    for (Map.Entry<Camera, List<PhotonTrackedTarget>> entry : tags.entrySet()) {
      var camera = entry.getKey();
      for (PhotonTrackedTarget tag : entry.getValue()) {
        var id = tag.getFiducialId();
        // x
        var pitchAngle = tag.getPitch();
        // z
        var yawAngle = tag.getYaw() + camera.yawOffset;
        var tagHeight = tagOffset.get(id);

        var tempxDist = (tagHeight - camera.zOffset) / Math.tan(yawAngle * Math.PI / 180);
        var yDist = (Math.tan(pitchAngle * Math.PI / 180) * tempxDist) + camera.xOffset;
        var xDist = tempxDist + camera.xOffset;

        out.put(Integer.valueOf(id), new Transform2d(new Translation2d(xDist, yDist)
            .rotateBy(Rotation2d.fromDegrees(robotAngle.getDegrees() + camera.pitchOffest)), Rotation2d.fromDegrees(0)));
      }
    }
    return out;
  }

  public List<Pose2d> getRobotPose(Rotation2d rotation) {
    var limelightOffsets = getDistances(rotation);
    List<Pose2d> poses = new ArrayList<>();
    for (Map.Entry<Integer, Transform2d> entry : limelightOffsets.entrySet()) {
      var id = entry.getKey();
      var offset = entry.getValue();

      poses.add(tagLocations.get(id).transformBy(offset.inverse()));
    }

    return poses;
  }

}
