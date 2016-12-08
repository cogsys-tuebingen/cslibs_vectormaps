#ifndef LAYERLISTITEM_HPP
#define LAYERLISTITEM_HPP

#include <QCheckBox>
#include <QColorDialog>
#include <QPushButton>
#include <QWidget>

#include <ui_map_viewer_list_item.h>

class QLayerListItem : public QWidget
{
    Q_OBJECT

public:
    QLayerListItem(QWidget * parent = 0) :
        QWidget(parent),
        ui_(new Ui::map_viewer_list_item),
        color_(Qt::black)
    {
        ui_->setupUi(this);

        updateColorSelection();

        connect(ui_->colorSelect, SIGNAL(clicked(bool)), this, SLOT(chooseColor()));
        connect(ui_->checkBox, SIGNAL(clicked(bool)), this, SLOT(changeVisibility()));
    }

    virtual ~QLayerListItem()
    {
    }

    void setName(const QString &name)
    {
        ui_->label->setText(name);
        name_ = name;
    }

    void setColor(const QColor &color)
    {
        color_ = color;
        updateColorSelection();
    }


    inline QString getName() const
    {
        return name_;
    }

    inline QColor getColor() const
    {
        return color_;
    }

    inline bool getVisibility() const
    {
        return ui_->checkBox->isChecked();
    }

signals:
    void hasChanged(QString name);

private:
    Ui::map_viewer_list_item *ui_;

    QColor   color_;
    QString  name_;

    void updateColorSelection()
    {
        QString s = "background-color: ";
        ui_->colorSelect->setStyleSheet(s + color_.name());
    }


private slots:
    void changeVisibility()
    {
        hasChanged(name_);
    }

    void chooseColor()
    {
        color_ = QColorDialog::getColor();
        updateColorSelection();
        hasChanged(name_);
    }
};


#endif // LAYERLISTITEM_HPP
