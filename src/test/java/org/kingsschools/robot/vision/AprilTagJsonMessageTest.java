package org.kingsschools.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.cyberknights4911.robot.vision.AprilTagJsonMessage;
import com.fasterxml.jackson.databind.ObjectMapper;

public class AprilTagJsonMessageTest {

    private String EXAMPLE_TAG = "{\"id\": 2, \"hamming\": 0, \"decision_margin\": " +
        "55.401695251464844, \"center\": [722.6058091498052, 337.72965839480423], \"corners\": " +
        "[[571.8601684570311, 502.69149780273443], [889.0171508789062, 491.29312133789057], " +
        "[876.870849609375, 168.91651916503906], [556.4097290039061, 184.3648376464844]], " +
        "\"pose_R\": [[0.02719742825504004, -0.2036835894523172, 0.9786589269424418], " +
        "[0.7196822756862508, 0.6834572004598715, 0.12224433403071976], [-0.6937706551587282, " +
        "0.7009987521582152, 0.16517574735188578]], \"pose_t\": [[0.04680199236991478], " +
        "[0.00970026867787437], [0.6765868549121506]]}";
    private AprilTagJsonMessage expectedTag;

    ObjectMapper mapper;

    @BeforeEach
    public void setup() {
        mapper = new ObjectMapper();

        double[][] corners = new double[4][2];
        corners[0][0] = 571.8601684570311;
        corners[0][1] = 502.69149780273443;
        corners[1][0] = 889.0171508789062;
        corners[1][1] = 491.29312133789057;
        corners[2][0] = 876.870849609375;
        corners[2][1] = 168.91651916503906;
        corners[3][0] = 556.4097290039061;
        corners[3][1] = 184.3648376464844;
        
        double[][] pose_R = new double[3][3];
        pose_R[0][0] = 0.02719742825504004;
        pose_R[0][1] = -0.2036835894523172;
        pose_R[0][2] = 0.9786589269424418;
        pose_R[1][0] = 0.7196822756862508;
        pose_R[1][1] = 0.6834572004598715;
        pose_R[1][2] = 0.12224433403071976;
        pose_R[2][0] = -0.6937706551587282;
        pose_R[2][1] = 0.7009987521582152;
        pose_R[2][2] = 0.16517574735188578;

        double[][] pose_t = new double[3][1];
        pose_t[0][0] = 0.04680199236991478;
        pose_t[1][0] = 0.00970026867787437;
        pose_t[2][0] = 0.6765868549121506;

        expectedTag = new AprilTagJsonMessage(
            2,
            0,
            55.401695251464844,
            new double[] { 722.6058091498052, 337.72965839480423 },
            corners,
            pose_R,
            pose_t);
    }

    @Test
    public void testParse() throws Exception {
        AprilTagJsonMessage parsedTag = mapper.readValue(EXAMPLE_TAG, AprilTagJsonMessage.class);
        assertEquals(expectedTag, parsedTag);
    }
}
