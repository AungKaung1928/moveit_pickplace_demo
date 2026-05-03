#pragma once

#include <cmath>
#include <string>

namespace moveit_grasp_utils {

/**
 * Lightweight geometric workspace check for the Franka Panda arm.
 *
 * Uses conservative cylindrical approximation:
 *   - horizontal reach: [min_reach, max_reach] from base Z-axis
 *   - height:           [min_height, max_height]
 *   - avoids the ~10° cone directly behind the base
 */
class WorkspaceValidator {
public:
    struct Params {
        double max_reach   = 0.82;
        double min_reach   = 0.18;
        double max_height  = 0.98;
        double min_height  = -0.05;
    };

    WorkspaceValidator() : p_(Params{}) {}
    explicit WorkspaceValidator(const Params & p) : p_(p) {}

    /**
     * Returns true if (x, y, z) in panda_link0 is inside the reachable workspace.
     * Also filters the singularity cone immediately above the base.
     */
    bool is_reachable(double x, double y, double z) const noexcept
    {
        if (z < p_.min_height || z > p_.max_height)
            return false;

        const double r = std::sqrt(x * x + y * y);
        if (r < p_.min_reach || r > p_.max_reach)
            return false;

        // Exclude narrow column directly above base (singularity risk)
        if (r < 0.08)
            return false;

        return true;
    }

    /**
     * Returns a human-readable rejection reason (for diagnostics).
     */
    std::string rejection_reason(double x, double y, double z) const
    {
        const double r = std::sqrt(x * x + y * y);
        if (z < p_.min_height) return "below min height";
        if (z > p_.max_height) return "above max height";
        if (r < 0.08)          return "too close to base (singularity)";
        if (r < p_.min_reach)  return "horizontal distance too small";
        if (r > p_.max_reach)  return "out of reach";
        return "unknown";
    }

    const Params & params() const { return p_; }

private:
    Params p_;
};

}  // namespace moveit_grasp_utils
