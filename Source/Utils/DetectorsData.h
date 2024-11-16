#pragma once

#include <opencv2/features2D.hpp>

// ORB data
const int ORB_NFEATURES = 200;             // Increase number of keypoints
const float ORB_SCALE_FACTOR = 1.2f;       // Adjust scale factor
const int ORB_NLEVELS = 3;                 // Number of pyramid levels
const int ORB_EDGE_THRESHOLD = 100;        // Lower edge threshold
const int ORB_FIRST_LEVEL = 0;             // First level of pyramid
const int ORB_WTA_K = 4;                   // BRIEF descriptor size
const cv::ORB::ScoreType ORB_SCORE_TYPE = cv::ORB::FAST_SCORE; // Scoring type
const int ORB_PATCH_SIZE = 15;             // Size of the patch
const int ORB_FAST_THRESHOLD = 20;         // FAST threshold

// AKAZE data
const cv::AKAZE::DescriptorType AKAZE_DESCRIPTOR_TYPE = cv::AKAZE::DESCRIPTOR_MLDB; // Type of descriptor (e.g., DESCRIPTOR_KAZE, DESCRIPTOR_MLDB)
const int AKAZE_DESCRIPTOR_SIZE = 0;       // Size of the descriptor (in bits). 0 means full size.
const int AKAZE_DESCRIPTOR_CHANNELS = 3;   // Number of descriptor channels (1, 2, or 3).
const float AKAZE_THRESHOLD = 0.001f;      // Detector response threshold. Lower values mean more keypoints.
const int AKAZE_NOCTAVES = 4;              // Number of pyramid octaves.
const int AKAZE_NOCTAVE_LAYERS = 4;        // Number of layers within each octave.
const cv::KAZE::DiffusivityType AKAZE_DIFFUSIVITY = cv::KAZE::DIFF_PM_G2; // Diffusivity function (e.g., DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT)

// BRISK data
const int BRISK_THRESH = 70;               // FAST/AGAST detection threshold score. Lower values mean more keypoints.
const int BRISK_OCTAVES = 1;               // Detection octaves (pyramid layers). Increase to detect features at more scales.
const float BRISK_PATTERN_SCALE = 1.0f;    // Scale of the pattern used for descriptor generation. Adjust to fine-tune keypoint localization.
