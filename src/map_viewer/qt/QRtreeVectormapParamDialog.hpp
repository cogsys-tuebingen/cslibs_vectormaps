#ifndef QRTREEVECTORMAPPARAMDIALOG_HPP
#define QRTREEVECTORMAPPARAMDIALOG_HPP

#include "../algorithms/rtree_vectormap_conversion.h"

#include <ui_map_viewer_rtree_vectormap_param_dialog.h>

#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
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
        ui_.comboBox_type->setCurrentText(QString::fromStdString(params.type));
        ui_.checkBox_discard_segments->setCheckState(params.discard_segments ? Qt::Checked : Qt::Unchecked);
    }

    void get(cslibs_vectormaps::RtreeVectormapConversionParameter& params)
    {
        QString path = ui_.lineEdit_path->text();
        if (!path.endsWith(QString(".gzip"))
        && !path.endsWith(QString(".yaml"))) {
            path += QString(".yaml");
        }
        params.path = path.toStdString();
        params.type = ui_.comboBox_type->currentText().toStdString();
        params.discard_segments = ui_.checkBox_discard_segments->checkState() == Qt::Checked;
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
