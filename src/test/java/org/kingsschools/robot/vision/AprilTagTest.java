package org.kingsschools.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.cyberknights4911.robot.vision.AprilTag;
import com.fasterxml.jackson.databind.ObjectMapper;

public class AprilTagTest {

    private String EXAMPLE_TAG = "{\"id\": 2, \"hamming\": 0, \"decision_margin\": 55.401695251464844, \"center\": [722.6058091498052, 337.72965839480423], \"corners\": [[571.8601684570311, 502.69149780273443], [889.0171508789062, 491.29312133789057], [876.870849609375, 168.91651916503906], [556.4097290039061, 184.3648376464844]], \"pose_t\": [[0.04680199236991478], [0.00970026867787437], [0.6765868549121506]]}";
    private AprilTag expectedTag;

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

        double[][] pose_t = new double[3][1];
        pose_t[0][0] = 0.04680199236991478;
        pose_t[1][0] = 0.00970026867787437;
        pose_t[2][0] = 0.6765868549121506;

        expectedTag = new AprilTag(
            2,
            0,
            55.401695251464844,
            new double[] { 722.6058091498052, 337.72965839480423 },
            corners,
            pose_t);
    }

    @Test
    public void testParse() throws Exception {
        AprilTag parsedTag = mapper.readValue(EXAMPLE_TAG, AprilTag.class);
        assertEquals(expectedTag, parsedTag);
    }
}
