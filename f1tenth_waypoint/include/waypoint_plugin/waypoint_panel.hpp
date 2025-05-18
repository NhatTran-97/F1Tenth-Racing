#ifndef F1TENTH_WAYPOINT_FOLLOWER__WAYPOINT_PANEL_HPP_
#define F1TENTH_WAYPOINT_FOLLOWER__WAYPOINT_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QVBoxLayout>

namespace f1tenth_waypoint_follower
{
    class WaypointPanel : public rviz_common::Panel
    {
    Q_OBJECT
    public:
        WaypointPanel(QWidget* parent = nullptr);
        virtual ~WaypointPanel() override = default;
    };
}

#endif  // F1TENTH_WAYPOINT_FOLLOWER__WAYPOINT_PANEL_HPP_
