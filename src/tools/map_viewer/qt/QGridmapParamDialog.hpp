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

    Pose getOrigin() const
    {
        return {
            ui_->doubleSpinBox_origin_x->value(),
            ui_->doubleSpinBox_origin_y->value(),
            ui_->doubleSpinBox_origin_phi->value() / 180. * M_PI
        };
    }

    double getResolution() const
    {
        return ui_->doubleSpinBox_resolution->value();
    }

    QString getPath() const
    {
        return ui_->lineEdit_path->text();
    }

private slots:
    void pushButton_path_pushed(bool)
    {
        QString file_name = QFileDialog::getExistingDirectory(this, "Save under ...", "~", QFileDialog::ShowDirsOnly);
        if(file_name != "")
            ui_->lineEdit_path->setText(file_name);
    }

private:
    Ui::GridmapParamDialog *ui_;

};

#endif // QGRIDMAPPARAMDIALOG_HPP
