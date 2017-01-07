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

    double getMinCornerAngle() const
    {
        return toRad(ui_->doubleSpinBox_min_corner_angle->value());
    }

    double getMaxCornerPointDistance() const
    {
        return ui_->doubleSpinBox_max_corner_distance->value();
    }

    double getMinLooseEndpointDistance() const
    {
        return ui_->doubleSpinBox_min_loose_endpoint_distance->value();
    }

    double getPrefCornerAngle() const
    {
        return toRad(ui_->doubleSpinBox_pref_corner_angle->value());
    }

    double getPrefCornerAngleStdDev() const
    {
        return toRad(ui_->doubleSpinBox_pref_corner_angle_std_dev->value());
    }

    inline double toRad(const double &deg) const
    {
        return deg / 180.0 * M_PI;
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
