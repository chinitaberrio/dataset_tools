/****************************************************************************
** Meta object code from reading C++ file 'datasetgui.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/DatasetGUI/datasetgui.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'datasetgui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DatasetGUI_t {
    QByteArrayData data[17];
    char stringdata0[217];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DatasetGUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DatasetGUI_t qt_meta_stringdata_DatasetGUI = {
    {
QT_MOC_LITERAL(0, 0, 10), // "DatasetGUI"
QT_MOC_LITERAL(1, 11, 12), // "startPressed"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 11), // "stopPressed"
QT_MOC_LITERAL(4, 37, 12), // "pausePressed"
QT_MOC_LITERAL(5, 50, 14), // "convertPressed"
QT_MOC_LITERAL(6, 65, 13), // "sliderPressed"
QT_MOC_LITERAL(7, 79, 11), // "sliderMoved"
QT_MOC_LITERAL(8, 91, 9), // "new_value"
QT_MOC_LITERAL(9, 101, 14), // "sliderReleased"
QT_MOC_LITERAL(10, 116, 22), // "changePlaybackRealtime"
QT_MOC_LITERAL(11, 139, 13), // "OnClickedTree"
QT_MOC_LITERAL(12, 153, 12), // "clicked_item"
QT_MOC_LITERAL(13, 166, 11), // "recordEvent"
QT_MOC_LITERAL(14, 178, 16), // "saveEventsToFile"
QT_MOC_LITERAL(15, 195, 7), // "PollROS"
QT_MOC_LITERAL(16, 203, 13) // "selectBagFile"

    },
    "DatasetGUI\0startPressed\0\0stopPressed\0"
    "pausePressed\0convertPressed\0sliderPressed\0"
    "sliderMoved\0new_value\0sliderReleased\0"
    "changePlaybackRealtime\0OnClickedTree\0"
    "clicked_item\0recordEvent\0saveEventsToFile\0"
    "PollROS\0selectBagFile"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DatasetGUI[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x09 /* Protected */,
       3,    0,   80,    2, 0x09 /* Protected */,
       4,    0,   81,    2, 0x09 /* Protected */,
       5,    0,   82,    2, 0x09 /* Protected */,
       6,    0,   83,    2, 0x09 /* Protected */,
       7,    1,   84,    2, 0x09 /* Protected */,
       9,    0,   87,    2, 0x09 /* Protected */,
      10,    0,   88,    2, 0x09 /* Protected */,
      11,    1,   89,    2, 0x09 /* Protected */,
      13,    0,   92,    2, 0x09 /* Protected */,
      14,    0,   93,    2, 0x09 /* Protected */,
      15,    0,   94,    2, 0x09 /* Protected */,
      16,    0,   95,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QModelIndex,   12,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void DatasetGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DatasetGUI *_t = static_cast<DatasetGUI *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->startPressed(); break;
        case 1: _t->stopPressed(); break;
        case 2: _t->pausePressed(); break;
        case 3: _t->convertPressed(); break;
        case 4: _t->sliderPressed(); break;
        case 5: _t->sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->sliderReleased(); break;
        case 7: _t->changePlaybackRealtime(); break;
        case 8: _t->OnClickedTree((*reinterpret_cast< QModelIndex(*)>(_a[1]))); break;
        case 9: _t->recordEvent(); break;
        case 10: _t->saveEventsToFile(); break;
        case 11: _t->PollROS(); break;
        case 12: _t->selectBagFile(); break;
        default: ;
        }
    }
}

const QMetaObject DatasetGUI::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_DatasetGUI.data,
      qt_meta_data_DatasetGUI,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *DatasetGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DatasetGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DatasetGUI.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int DatasetGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
