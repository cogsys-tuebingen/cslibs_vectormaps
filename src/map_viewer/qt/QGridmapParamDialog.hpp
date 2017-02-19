#ifndef QGRIDMAPPARAMDIALOG_HPP
#define QGRIDMAPPARAMDIALOG_HPP


#include <QDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <ui_map_viewer_gridmap_param_dialog.h>
#include <QFileDialog>
#include <array>

class QGridmapParamDialog : public QDialog {

    Q_OBJECT

public:
    using Pose = std::array<double, 3>;

    QGridmapParamDialog() :
        ui_(new Ui::GridmapParamDialog)
    {
        ui_->setupUi(this);

        connect(ui_->pushButton_path, SIGNAL(clicked(bool)),
                this, SLOT(pushButton_path_pushed(bool)));
    }

    virtual ~QGridmapParamDialog()
    {
        delete ui_;
    }

    void setup(const Pose    &origin,
               const double   resolution,
               const QString &path)
    {
        ui_->doubleSpinBox_origin_x->setValue(origin[0]);
        ui_->doubleSpinBox_origin_y->setValue(origin[1]);
        ui_->doubleSpinBox_origin_phi->setValue(toDeg(origin[2]));

        ui_->doubleSpinBox_resolution->setValue(resolution);

        ui_->lineEdit_path->setText(path);
    }

    void get(Pose    &origin,
             double  &resolution,
             QString &path)
    {
        origin[0] = ui_->doubleSpinBox_origin_x->value();
        origin[1] = ui_->doubleSpinBox_origin_y->value();
        origin[2] = ui_->doubleSpinBox_origin_phi->value();

        resolution = ui_->doubleSpinBox_resolution->value();

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
        QString file_name = QFileDialog::getSaveFileName(this, "Save under ...", ui_->lineEdit_path->text());
        if(file_name != "")
            ui_->lineEdit_path->setText(file_name);
    }

private:
    Ui::GridmapParamDialog *ui_;

};

#endif // QGRIDMAPPARAMDIALOG_HPP
