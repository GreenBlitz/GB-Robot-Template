package frc.utils;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class JsonConverter {

    private static final ObjectMapper objectMapper = new ObjectMapper();

    public static <T> T objectFromJson(String content, Class<T> classOfObject) {
        try {
            return objectMapper.readValue(content, classOfObject);
        }
        catch (JsonProcessingException ignored) {}
        return null;
    }

    public static String readFromObject(Object object) {
        try {
            return objectMapper.writeValueAsString(object);
        }
        catch (JsonProcessingException ignored) {}
        return "error - couldn't read from the object";
    }

}
