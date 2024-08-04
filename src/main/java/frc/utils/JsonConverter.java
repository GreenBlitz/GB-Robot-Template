package frc.utils;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.poseestimator.OdometryObservation;
import frc.robot.poseestimator.VisionObservation;

public class JsonConverter {
    private static final ObjectMapper objectMapper = new ObjectMapper();

    public static VisionObservation visionObservationFromJson(String content) {
        return objectFromJson(content, VisionObservation.class);
    }

    public static OdometryObservation odometryObservationFromJson(String content) {
        return objectFromJson(content, OdometryObservation.class);
    }

    public static <T> T objectFromJson(String content, Class<T> classofObject) {
        try {
            return objectMapper.readValue(content, classofObject);
        }
        catch (JsonProcessingException e) {
            throw new RuntimeException(e);
        }
    }

    public static String readFromObject(Object object) {
        try {
            return objectMapper.writeValueAsString(object);
        }
        catch (JsonProcessingException e) {
            throw new RuntimeException(e);
        }
    }

}
