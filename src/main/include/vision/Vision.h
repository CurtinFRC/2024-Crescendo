#pragma once

#include <units/length.h>
#include <string>
#include "Wombat.h"
#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/json.h>
#include <units/math.h>

#define APRILTAGS_MAX 16
#define APRILTAGS_MIN 0

enum class VisionTarget {
    kNote = 0,
    kAmp = 1,
    kSource = 2,
    kNone = 3
};

struct AprilTag {
    int id;
    double size;
    std::array<std::array<double, 4>, 4> transform;
    bool unique;
};

class FMAP {
    public:
        FMAP(std::string path);

        std::vector<AprilTag> GetTags();

    private:
        std::vector<AprilTag> _tags;
        std::string _path;
        std::string deploy_dir;
};

class Vision {
    public:
        Vision(std::string name, FMAP fmap);

        std::pair<units::meter_t, units::degree_t> GetDistanceToTarget(VisionTarget target);
        std::pair<units::meter_t, units::degree_t> GetDistanceToTarget(int id);

        frc::Pose3d GetPose();

        frc::Pose2d AlignToTarget(VisionTarget target, wom::SwerveDrive* swerveDrive);

        std::vector<AprilTag> GetTags();

    private:
        wom::Limelight* _limelight;    
        FMAP _fmap;
};