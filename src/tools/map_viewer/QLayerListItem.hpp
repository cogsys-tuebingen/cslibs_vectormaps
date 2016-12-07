#ifndef LAYERLISTITEM_HPP
#define LAYERLISTITEM_HPP

#include <QWidget>
#include <QColorDialog>
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

        palette_.setColor(QPalette::Button, color_);
        ui_->colorSelect->setPalette(palette_);

        connect(ui_->colorSelect, SIGNAL(clicked(bool)), this, SLOT(getColor()));
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
        palette_.setColor(QPalette::Button, color_);
        ui_->colorSelect->setPalette(palette_);
    }


    inline QString getName() const
    {
        return name_;
    }

    inline QColor getColor() const
    {
        return color_;
    }

    inline bool isVisible() const
    {
        return ui_->checkBox->isChecked();
    }

private:
    Ui::map_viewer_list_item *ui_;

    QPalette palette_;
    QColor   color_;
    QString  name_;


private slots:
    void getColor()
    {
        QColorDialog d;
        d.open(this, SLOT(colorSelected(QColor)));
    }

    void colorSelected(const QColor &color)
    {
        color_ = color;
    }

};


#endif // LAYERLISTITEM_HPP
