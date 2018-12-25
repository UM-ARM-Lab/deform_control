#ifndef QT_ROS_TEST_NODE_H
#define QT_ROS_TEST_NODE_H

#include <QWidget>
#include <QPushButton>

class Window : public QWidget
{
    Q_OBJECT

public:
    explicit Window(QWidget* parent = nullptr);

    static constexpr int EXIT_RESTART = -50;

private slots:
    void restartSlot();

private:
    QPushButton* restart_button_;
};

#endif // QT_ROS_TEST_NODE_H
