#ifndef QDOORPARAMDIALOG_HPP
#define QDOORPARAMDIALOG_HPP

#include "../algorithms/find_doors.h"

#include <ui_map_viewer_door_param_dialog.h>

#include <QObject>
#include <QDialog>

#include <boost/math/constants/constants.hpp>

class QDoorParamDialog : public QDialog {
    Q_OBJECT

public:
    QDoorParamDialog()
    {
        ui_.setupUi(this);
    }

    void setup(const cslibs_vectormaps::FindDoorsParameter& params)
    {
        ui_.doubleSpinBox_merge_max_proximity->setValue(params.merge_max_proximity);
        ui_.doubleSpinBox_door_depth_min->setValue(params.door_depth_min);
        ui_.doubleSpinBox_door_depth_max->setValue(params.door_depth_max);
        ui_.doubleSpinBox_door_width_min->setValue(params.door_width_min);
        ui_.doubleSpinBox_door_width_max->setValue(params.door_width_max);
        ui_.doubleSpinBox_door_angle_diff_max->setValue(toDeg(params.door_angle_diff_max));
    }

    void get(cslibs_vectormaps::FindDoorsParameter& params)
    {
        params.merge_max_proximity = ui_.doubleSpinBox_merge_max_proximity->value();
        params.door_depth_min = ui_.doubleSpinBox_door_depth_min->value();
        params.door_depth_max = ui_.doubleSpinBox_door_depth_max->value();
        params.door_width_min = ui_.doubleSpinBox_door_width_min->value();
        params.door_width_max = ui_.doubleSpinBox_door_width_max->value();
        params.door_angle_diff_max = toRad(ui_.doubleSpinBox_door_angle_diff_max->value());
    }

    inline double toRad(double deg) const
    {
        return deg * boost::math::double_constants::pi / 180.0;
    }

    inline double toDeg(double rad) const
    {
        return rad * 180.0 / boost::math::double_constants::pi;
    }

private:
    Ui::DoorParamDialog ui_;
};

#endif // QDOORPARAMDIALOG_HPP
