#ifndef QVECTORMAPPARAMDIALOG_HPP
#define QVECTORMAPPARAMDIALOG_HPP


#include <QDialog>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <ui_map_viewer_vectormap_param_dialog.h>
#include <QFileDialog>
#include <array>

class QVectormapParamDialog : public QDialog {

    Q_OBJECT

public:
    using Pose = std::array<double, 3>;

    QVectormapParamDialog() :
        ui_(new Ui::VectormapParamDialog)
    {
        ui_->setupUi(this);

        connect(ui_->pushButton_path, SIGNAL(clicked(bool)),
                this, SLOT(pushButton_path_pushed(bool)));
    }

    virtual ~QVectormapParamDialog()
    {
        delete ui_;
    }

    void setup(const double     angular_resolution,
               const double     linear_resolution,
               const double     range,
               const QString   &type,
               const QString   &path)
    {
        ui_->doubleSpinBox_ang_resolution->setValue(toDeg(angular_resolution));
        ui_->doubleSpinBox_lin_resolution->setValue(linear_resolution);
        ui_->doubleSpinBox_range->setValue(range);
        ui_->comboBox_type->setCurrentText(type);
        ui_->lineEdit_path->setText(path);
    }

    void get(double  &angular_resolution,
             double  &linear_resolution,
             double  &range,
             QString &type,
             QString &path)
    {
        angular_resolution = toRad(ui_->doubleSpinBox_ang_resolution->value());
        linear_resolution  = ui_->doubleSpinBox_lin_resolution->value();
        range = ui_->doubleSpinBox_range->value();
        type = ui_->comboBox_type->currentText();
        path = ui_->lineEdit_path->text();
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
    void pushButton_path_pushed(bool)
    {
        QString file_name = QFileDialog::getSaveFileName(this, "Save under ...", ui_->lineEdit_path->text(), "*.gzip,*.yaml");
        if(file_name != "")
            ui_->lineEdit_path->setText(file_name);
    }

private:
    Ui::VectormapParamDialog *ui_;

};


#endif // QVECTORMAPPARAMDIALOG_HPP
