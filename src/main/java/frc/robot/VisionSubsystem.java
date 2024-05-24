// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.FieldConstants;

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
    // forward-backward
    xOffset = xoffset;
    // side-side
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

  List<Camera> cameras = new ArrayList<>();

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
        
        Rotation2d rotation = new Rotation2d(robotAngle.getDegrees() + getYawAngle(tag, camera));
        Translation2d translation = new Translation2d(getDistanceMagnitude(tag, camera), rotation);
        translation = translation.plus(new Translation2d(camera.xOffset, camera.yOffset).rotateBy(rotation));
        
        out.put(Integer.valueOf(id), new Transform2d(translation, Rotation2d.fromDegrees(0)));
      }
    }
    return out;
  }

  public HashMap<Integer, Double> getDistancesMagnitudes () {
    HashMap<Integer, Double> distances = new HashMap<>();
    for (Map.Entry<Camera, List<PhotonTrackedTarget>> entry : tags.entrySet()) {
      for (PhotonTrackedTarget tag : entry.getValue()) {
          distances.put(tag.getFiducialId(), getDistanceMagnitude(tag, entry.getKey()));
      }
    }
    return distances;
  }

  public Double[] getDistanceMagnitudesList () {
    List<Double> distances = new ArrayList<>();
     for (Map.Entry<Camera, List<PhotonTrackedTarget>> entry : tags.entrySet()) {
      for (PhotonTrackedTarget tag : entry.getValue()) {
          distances.add(getDistanceMagnitude(tag, entry.getKey()));
      }
    }
    Double[] out = new Double[distances.size()];
    for (int i = 0; i < out.length; i++) {
      out[i] = distances.get(i);
    }
    return out;

  }

  public HashMap<Integer, Double> getYawAngles () {
    HashMap<Integer, Double> angles = new HashMap<>();
    for (Map.Entry<Camera, List<PhotonTrackedTarget>> entry : tags.entrySet()) {
      for (PhotonTrackedTarget tag : entry.getValue()) {
          angles.put(tag.getFiducialId(), getYawAngle(tag, entry.getKey()));
      }
    }
    return angles;
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

  public double getDistanceMagnitude(PhotonTrackedTarget target, Camera camera) {
    var tagHeight = tagOffset.get(target.getFiducialId());
    var tempxDist = (tagHeight - camera.zOffset) / Math.tan((target.getPitch() + camera.pitchOffest) * Math.PI / 180);
    return tempxDist;
  } 


  public double getYawAngle (PhotonTrackedTarget target, Camera camera) {
    return target.getYaw() + camera.yawOffset;
  }

  public double getSpeakerTargetDistance() {
    for (Map.Entry<Integer, Double> entry : getDistancesMagnitudes().entrySet()) {
      if (entry.getKey() == FieldConstants.kBlueSpeakerTagID || entry.getKey() == FieldConstants.kRedSpeakerTagID) {
        return entry.getValue().doubleValue();
      }
    }
    return Double.NaN;
  }

  public double getSpeakerTargetYaw() {
    for (Map.Entry<Integer, Double> entry : getYawAngles().entrySet()) {
      if (entry.getKey() == FieldConstants.kBlueSpeakerTagID || entry.getKey() == FieldConstants.kRedSpeakerTagID) {
        return entry.getValue().doubleValue();
      }
    }
    return Double.NaN;
  }

  


}
