package com.cyberknights4911.robot.vision;

import java.util.Arrays;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * A simple data class for representing AprilTags.
 * 
 * Actual json format:
 * {
 *     "id": 6, 
 *     "hamming": 2, 
 *     "decision_margin": 13.702051162719727, 
 *     "center": [-355.01943715483964, 989.6513128560938], 
 *     "corners": [[159.31039428710938, 601.7501831054688], [54.83549880981428, 670.8073730468751], [170.05258178710938, 593.6485595703125], [149.46437072753895, 597.1914672851564]], 
 *     "pose_t": [[-0.5612786884816965], [0.34145442852213037], [1.6247312388643178]]
 * }
 */
public final class AprilTagJsonMessage {
    // These are all the tag ids used in the game
    public static final int ID00 = 0;
    public static final int ID01 = 1;
    public static final int ID02 = 2;
    public static final int ID03 = 3;
    public static final int ID04 = 4;
    public static final int ID05 = 5;
    public static final int ID06 = 6;
    public static final int ID07 = 7;

    private final int id;
    
    private final int hamming;
    
    private final double decision_margin;

    private final double[] center;

    private final double[][] corners;

    private final double[][] pose_R;
    
    private final double[][] pose_t;

    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public AprilTagJsonMessage(
            @JsonProperty("id") int id,
            @JsonProperty("hamming") int hamming,
            @JsonProperty("decision_margin") double decision_margin,
            @JsonProperty("center") double[] center,
            @JsonProperty("corners") double[][] corners,
            @JsonProperty("pose_R") double[][] pose_R,
            @JsonProperty("pose_t") double[][] pose_t) {
        this.id = id;
        this.hamming = hamming;
        this.decision_margin = decision_margin;
        this.center = center;
        this.corners = corners;
        this.pose_R = pose_R;
        this.pose_t = pose_t;
    }

    /** The decoded ID of the tag. */
    public int getId() {
        return id;
    }

    /**
     * How many error bits were corrected? Note: accepting large numbers of corrected errors 
     * leads to greatly increased false positive rates.
     */
    public int getHamming() {
        return hamming;
    }

    /**
     * A measure of the quality of the binary decoding process: the average difference between the
     * intensity of a data bit versus the decision threshold. Higher numbers roughly indicate
     * better decodes. This is a reasonable measure of detection accuracy only for very small tags
     * --not effective for larger tags (where we could have sampled anywhere within a bit cell and
     * still gotten a good detection.)
     * @return
     */
    public double getDecision_margin() {
        return decision_margin;
    }

    /** The center of the detection in image pixel coordinates. */
    public double[] getCenter() {
        return center;
    }

    /**
     * The corners of the tag in image pixel coordinates. These always wrap counter-clock wise
     * around the tag.
     */
    public double[][] getCorners() {
        return corners;
    }

    /** Rotation matrix of the pose estimate. */
    public double[][] getPose_R() {
        return pose_R;
    }

    /** Translation of the pose estimate. */
    public double[][] getPose_t() {
        return pose_t;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        AprilTagJsonMessage other = (AprilTagJsonMessage) obj;
        if (id != other.id)
            return false;
        if (hamming != other.hamming)
            return false;
        if (Double.doubleToLongBits(decision_margin) != Double.doubleToLongBits(other.decision_margin))
            return false;
        if (!Arrays.equals(center, other.center))
            return false;
        if (!Arrays.deepEquals(corners, other.corners))
            return false;
        if (!Arrays.deepEquals(pose_R, other.pose_R))
            return false;
        if (!Arrays.deepEquals(pose_t, other.pose_t))
            return false;
        return true;
    }
    
    /**
     * Creates an instance AprilTag object from this message.
     * @return the AprilTag
     */
    public AprilTag convertToWpiAprilTag() {
        Matrix<N3, N3> camMatrix = new Matrix<N3, N3>(Nat.N3(), Nat.N3());

        // pose_R is a 3x3 "camera matrix"
        for (int i = 0; i < pose_R.length; i++) {
            double[] row = pose_R[i];
            for (int j = 0; j < row.length; j++) {
                camMatrix.set(i, j, pose_R[i][j]);
            }
        }

        return new AprilTag(
            id,
            new Pose3d(
                new Translation3d(pose_t[0][0], pose_t[1][0], pose_t[2][0]),
                new Rotation3d(camMatrix)
            )
        );
    }
}
