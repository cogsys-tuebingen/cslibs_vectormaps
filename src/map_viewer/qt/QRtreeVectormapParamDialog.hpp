#ifndef QRTREEVECTORMAPPARAMDIALOG_HPP
#define QRTREEVECTORMAPPARAMDIALOG_HPP

#include "../algorithms/rtree_vectormap_conversion.h"

#include <boost/math/constants/constants.hpp>

#include <QDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <ui_map_viewer_rtree_vectormap_param_dialog.h>
#include <QFileDialog>

class QRtreeVectormapParamDialog : public QDialog {
    Q_OBJECT

public:
    QRtreeVectormapParamDialog()
    {
        ui_.setupUi(this);

        connect(ui_.pushButton_path, SIGNAL(clicked(bool)),
                this, SLOT(pushButton_path_pushed(bool)));
    }

    void setup(const cslibs_vectormaps::RtreeVectormapConversionParameter& params)
    {
        ui_.lineEdit_path->setText(QString::fromStdString(params.path));
        ui_.doubleSpinBox_merge_max_proximity->setValue(params.merge_max_proximity);
        ui_.doubleSpinBox_door_depth_min->setValue(params.door_depth_min);
        ui_.doubleSpinBox_door_depth_max->setValue(params.door_depth_max);
        ui_.doubleSpinBox_door_width_min->setValue(params.door_width_min);
        ui_.doubleSpinBox_door_width_max->setValue(params.door_width_max);
        ui_.doubleSpinBox_door_angle_diff_max->setValue(params.door_angle_diff_max);
    }

    void get(cslibs_vectormaps::RtreeVectormapConversionParameter& params)
    {
        QString path = ui_.lineEdit_path->text();
        if (!path.endsWith(QString(".gzip"))
        && !path.endsWith(QString(".yaml"))) {
            path += QString(".yaml");
        }
        params.path = path.toStdString();
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

private slots:
    void pushButton_path_pushed(bool)
    {
        QString file_name = QFileDialog::getSaveFileName(this, "Save under ...", ui_.lineEdit_path->text(), "*.gzip,*.yaml");
        if (file_name != "")
            ui_.lineEdit_path->setText(file_name);
    }

private:
    Ui::RtreeVectormapParamDialog ui_;
};

#endif // QRTREEVECTORMAPPARAMDIALOG_HPP
