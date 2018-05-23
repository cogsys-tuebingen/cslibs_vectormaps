#ifndef QROOMPARAMDIALOG_HPP
#define QROOMPARAMDIALOG_HPP

#include "../algorithms/find_rooms.h"

#include <ui_map_viewer_room_param_dialog.h>

#include <QObject>
#include <QDialog>

class QRoomParamDialog : public QDialog {
    Q_OBJECT

public:
    QRoomParamDialog()
    {
        ui_.setupUi(this);
    }

    void setup(const cslibs_vectormaps::FindRoomsParameter& params)
    {
        ui_.doubleSpinBox_merge_max_proximity->setValue(params.merge_max_proximity);
        ui_.doubleSpinBox_connect_max_proximity->setValue(params.connect_max_proximity);
    }

    void get(cslibs_vectormaps::FindRoomsParameter& params)
    {
        params.merge_max_proximity = ui_.doubleSpinBox_merge_max_proximity->value();
        params.connect_max_proximity = ui_.doubleSpinBox_connect_max_proximity->value();
    }

private:
    Ui::RoomParamDialog ui_;
};

#endif // QROOMPARAMDIALOG_HPP
