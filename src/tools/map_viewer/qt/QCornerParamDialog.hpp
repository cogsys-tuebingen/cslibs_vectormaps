#ifndef QCORNERPARAMDIALOG_HPP
#define QCORNERPARAMDIALOG_HPP

#include <QDialog>
#include <ui_map_viewer_corner_param_dialog.h>

class QCornerParamDialog : public QDialog {

    Q_OBJECT

public:
    QCornerParamDialog() :
        ui_(new Ui::CornerParamDialog)
    {
        ui_->setupUi(this);
        ui_->doubleSpinBox_min_angle->setMinimum(-M_PI);
        ui_->doubleSpinBox_min_angle->setMaximum(M_PI);

        connect(ui_->doubleSpinBox_max_distance, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_max_distance_changed(double)));
        connect(ui_->doubleSpinBox_min_angle, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_min_angle_changed(double)));
    }

    virtual ~QCornerParamDialog()
    {
        delete ui_;
    }

    double getMinLineAngle() const
    {
        return min_line_angle_;
    }

    double getMaxPointDistance() const
    {
        return max_point_distance_;
    }

private slots:
    void doubleSpinBox_max_distance_changed(const double value)
    {
        max_point_distance_ = value;
    }

    void doubleSpinBox_min_angle_changed(const double value)
    {
        min_line_angle_ = value;
    }

private:
    Ui::CornerParamDialog *ui_;

    double min_line_angle_;
    double max_point_distance_;


};


#endif // QCORNERPARAMDIALOG_HPP
