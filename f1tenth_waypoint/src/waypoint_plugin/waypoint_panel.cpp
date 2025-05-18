#include "f1tenth_waypoint_follower/waypoint_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace f1tenth_waypoint_follower
{
    WaypointPanel::WaypointPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        // Implementation details...
    }

    // Destructor and other methods if necessary...
}

PLUGINLIB_EXPORT_CLASS(f1tenth_waypoint_follower::WaypointPanel, rviz_common::Panel)
