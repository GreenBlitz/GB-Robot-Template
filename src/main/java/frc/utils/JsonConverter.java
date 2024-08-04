package frc.utils;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class JsonConverter {
    private static final ObjectMapper objectMapper = new ObjectMapper();

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
