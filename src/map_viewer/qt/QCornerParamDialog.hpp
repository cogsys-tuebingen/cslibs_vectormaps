#ifndef QCORNERPARAMDIALOG_HPP
#define QCORNERPARAMDIALOG_HPP

#include <QDialog>
#include <QDoubleSpinBox>
#include <ui_map_viewer_corner_param_dialog.h>

class QCornerParamDialog : public QDialog {

    Q_OBJECT

public:
    QCornerParamDialog() :
        ui_(new Ui::CornerParamDialog)
    {
        ui_->setupUi(this);

        connect(ui_->doubleSpinBox_min_corner_angle, SIGNAL(valueChanged(double)),
                this, SLOT(doubleSpinBox_min_corner_angle(double)));
    }

    virtual ~QCornerParamDialog()
    {
        delete ui_;
    }

    void setup(const double min_corner_angle,
               const double max_corner_point_distance,
               const double min_loose_end_point_distance,
               const double pref_corner_angle,
               const double pref_corner_angle_std_dev)
    {
        ui_->doubleSpinBox_min_corner_angle->setValue(
                    toDeg(min_corner_angle));
        ui_->doubleSpinBox_max_corner_distance->setValue(
                    max_corner_point_distance);
        ui_->doubleSpinBox_min_loose_endpoint_distance->setValue(
                    min_loose_end_point_distance);
        ui_->doubleSpinBox_pref_corner_angle->setValue(
                    toDeg(pref_corner_angle));
        ui_->doubleSpinBox_pref_corner_angle_std_dev->setValue(
                    toDeg(pref_corner_angle_std_dev));
    }

    void get(double &min_corner_angle,
             double &max_corner_point_distance,
             double &min_loose_end_point_distance,
             double &pref_corner_angle,
             double &pref_corner_angle_std_dev)
    {
        min_corner_angle             = toRad(ui_->doubleSpinBox_min_corner_angle->value());
        max_corner_point_distance    = ui_->doubleSpinBox_max_corner_distance->value();
        min_loose_end_point_distance = ui_->doubleSpinBox_min_loose_endpoint_distance->value();
        pref_corner_angle            = toRad(ui_->doubleSpinBox_pref_corner_angle->value());
        pref_corner_angle_std_dev    = toRad(ui_->doubleSpinBox_pref_corner_angle_std_dev->value());
    }

    inline double toRad(const double &deg) const
    {
        return deg / 180.0 * M_PI;
    }

    inline double toDeg(const double &rad) const
    {
        return rad * 180.0 / M_PI;
    }

private slots:
    void doubleSpinBox_min_corner_angle(const double value)
    {
        ui_->doubleSpinBox_pref_corner_angle->setMinimum(value);
    }

private:
    Ui::CornerParamDialog *ui_;
};


#endif // QCORNERPARAMDIALOG_HPP
