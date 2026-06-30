#include "demo_cpp_pkg/tiaozi_gui.h"
#include <QApplication>
#include <cstdlib>
#include <signal.h>
#include <iostream>

void signal_handler(int sig) {
    rclcpp::shutdown();
    QApplication::exit(1);
}

int main(int argc, char *argv[])
{
    std::cout << "[DEBUG] main: Start" << std::endl;
    
    const char* domain_id = getenv("ROS_DOMAIN_ID");
    if (domain_id == nullptr) {
        setenv("ROS_DOMAIN_ID", "36", 1);
        domain_id = "36";
    }
    std::cout << "[DEBUG] main: ROS_DOMAIN_ID=" << domain_id << std::endl;

    QApplication a(argc, argv);
    std::cout << "[DEBUG] main: QApplication created" << std::endl;
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    std::cout << "[DEBUG] main: signal handlers set" << std::endl;

    try {
        std::cout << "[DEBUG] main: Creating HuataiControlWidget..." << std::endl;
        HuataiControlWidget w;
        std::cout << "[DEBUG] main: HuataiControlWidget created" << std::endl;
        w.setWindowTitle("协同调姿控制器");
        std::cout << "[DEBUG] main: setWindowTitle done" << std::endl;
        w.show();
        std::cout << "[DEBUG] main: show() done, entering event loop..." << std::endl;
        return a.exec();
    } catch (const std::exception& e) {
        fprintf(stderr, "启动异常: %s\n", e.what());
        return 1;
    } catch (...) {
        fprintf(stderr, "启动异常: 未知错误\n");
        return 1;
    }
}