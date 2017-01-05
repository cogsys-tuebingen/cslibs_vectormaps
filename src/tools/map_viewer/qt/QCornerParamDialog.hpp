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

        connect(ui_->doubleSpinBox_max_corner_distance, SIGNAL(valueChanged(double)),
                this, SLOT(doubleSpinBox_max_corner_distance(double)));
        connect(ui_->doubleSpinBox_min_corner_angle, SIGNAL(valueChanged(double)),
                this, SLOT(doubleSpinBox_min_corner_angle(double)));
        connect(ui_->doubleSpinBox_min_loose_endpoint_distance, SIGNAL(valueChanged(double)),
                this, SLOT(doubleSpinBox_min_loose_endpoint_distance_changed(double)));
        connect(ui_->doubleSpinBox_pref_corner_angle, SIGNAL(valueChanged(double)),
                this, SLOT(doubleSpinBox_pref_corner_angle_changed(double)));
        connect(ui_->doubleSpinBox_pref_corner_angle_std_dev, SIGNAL(valueChanged(double)),
                this, SLOT(doubleSpinBox_pref_corner_angle_std_dev_changed(double)));

        max_corner_point_distance_   = ui_->doubleSpinBox_max_corner_distance->value();
        min_corner_angle_            = toRad(ui_->doubleSpinBox_min_corner_angle->value());
        min_loose_endpoint_distance_ = ui_->doubleSpinBox_min_loose_endpoint_distance->value();
        pref_corner_angle_           = toRad(ui_->doubleSpinBox_pref_corner_angle->value());
        pref_corner_angle_std_dev_   = ui_->doubleSpinBox_pref_corner_angle_std_dev->value();
    }

    virtual ~QCornerParamDialog()
    {
        delete ui_;
    }

    double getMinCornerAngle() const
    {
        return min_corner_angle_;
    }

    double getMaxCornerPointDistance() const
    {
        return max_corner_point_distance_;
    }

    double getMinLooseEndpointDistance() const
    {
        return min_loose_endpoint_distance_;
    }

    double getPrefCornerAngle() const
    {
        return pref_corner_angle_;
    }

    double getPrefCornerAngleStdDev() const
    {
        return pref_corner_angle_std_dev_;
    }

    inline double toRad(const double &deg)
    {
        return deg / 180.0 * M_PI;
    }

private slots:
    void doubleSpinBox_max_corner_distance(const double value)
    {
        max_corner_point_distance_ = value;
    }

    void doubleSpinBox_min_corner_angle(const double value)
    {
        ui_->doubleSpinBox_pref_corner_angle->setMinimum(value);
        min_corner_angle_ = toRad(value);
    }

    void doubleSpinBox_min_loose_endpoint_distance_changed(const double value)
    {
        min_loose_endpoint_distance_ = value;
    }

    void doubleSpinBox_pref_corner_angle_changed(const double value)
    {
        pref_corner_angle_ = toRad(value);
    }

    void doubleSpinBox_pref_corner_angle_std_dev_changed(const double value)
    {
        pref_corner_angle_std_dev_ = toRad(value);
    }



private:
    Ui::CornerParamDialog *ui_;

    double min_corner_angle_;
    double max_corner_point_distance_;
    double min_loose_endpoint_distance_;
    double pref_corner_angle_;
    double pref_corner_angle_std_dev_;

};


#endif // QCORNERPARAMDIALOG_HPP
