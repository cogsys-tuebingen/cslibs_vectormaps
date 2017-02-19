#ifndef LAYERLISTITEM_HPP
#define LAYERLISTITEM_HPP

#include <QCheckBox>
#include <QColorDialog>
#include <QPushButton>
#include <QWidget>
#include <QLabel>

#include <ui_map_viewer_list_item.h>

namespace cslibs_vectormaps {
class LayerModel;
}

class QLayerListItem : public QWidget
{
    Q_OBJECT

public:
    using ModelPtr = typename std::shared_ptr<cslibs_vectormaps::LayerModel>;

    QLayerListItem(QWidget * parent = 0) :
        QWidget(parent),
        ui_(new Ui::map_viewer_list_item)
    {
        ui_->setupUi(this);
        connect(ui_->colorSelect, SIGNAL(clicked(bool)), this, SLOT(setColor()));
        connect(ui_->checkBox, SIGNAL(clicked(bool)), this, SLOT(setVisibility()));
    }

    virtual ~QLayerListItem()
    {
        delete ui_;
    }

    void setModel(const ModelPtr &model)
    {
        model_ = model;
        ui_->checkBox->setChecked(model_->getVisibility());
        ui_->label->setText(model_->getName<QString>());
        updateColorSelection(model_->getColor());
    }


signals:
    void hasChanged(QString name);

private:
    Ui::map_viewer_list_item    *ui_;
    ModelPtr                     model_;

    void updateColorSelection(const QColor &color)
    {
        QString s = "background-color: ";
        ui_->colorSelect->setStyleSheet(s + color.name());
    }


private slots:
    void setVisibility()
    {
        model_->setVisible(ui_->checkBox->isChecked());
        hasChanged(model_->getName<QString>());
    }

    void setColor()
    {
        model_->setColor(QColorDialog::getColor());
        updateColorSelection(model_->getColor());
        hasChanged(model_->getName<QString>());
    }
};


#endif // LAYERLISTITEM_HPP
