#include <QApplication>
#include <QPushButton>
#include "qt_ros_test_node.h"

static QApplication* qt_app = nullptr;
static constexpr int EXIT_RESTART = -50;

Window::Window(QWidget* parent)
    : QWidget(parent)
{
    setFixedSize(100, 50);
    restart_button_ = new QPushButton("Hello world", this);
    restart_button_->setGeometry(10, 10, 80, 30);
    restart_button_->setCheckable(true);
    connect(restart_button_, SIGNAL(pressed()), this, SLOT(restartSlot()));
}

void Window::restartSlot()
{
    qt_app->exit(EXIT_RESTART);
}

int main(int argc, char* argv[])
{
    int current_exit_code = 0;
    do
    {
        // Start the QT managed window
        QApplication qt(argc, argv);
        qt_app = &qt;
        Window win;
        win.show();
        current_exit_code = qt.exec();
    }
    while (current_exit_code == EXIT_RESTART);

    return current_exit_code;
}
