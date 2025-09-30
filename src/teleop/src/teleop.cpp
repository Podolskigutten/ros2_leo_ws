#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cctype>
#include <algorithm> // std::clamp

// Put terminal in raw, non-blocking mode and restore on exit (RAII).
struct RawTerminal
{
    termios orig{};
    bool ok{false};
    RawTerminal()
    {
        if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &orig) == 0)
        {
            termios raw = orig;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0; // non-blocking
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
            ok = true;
        }
    }
    ~RawTerminal()
    {
        if (ok)
            tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    RawTerminal tty; // switches terminal to raw; auto-restores on exit

    //
    // Simple controls (tweak if you like)
    double v = 0.0, w = 0.0;             // linear.x, angular.z
    const double dv = 0.1, dw = 0.10;    // step per keypress
    const double vmax = 1.0, wmax = 1.0; // clamps
    rclcpp::WallRate rate(10);           // publish at xx Hz
    //
    //

    RCLCPP_INFO(node->get_logger(),
                "W/S: +/- linear.x | A/D: +/- angular.z | SPACE: stop | Q: quit\n"
                "Publishing geometry_msgs/Twist on /cmd_vel");

    bool quit = false;
    while (rclcpp::ok() && !quit)
    {
        // Read all pending keypresses (non-blocking)
        unsigned char ch;
        while (read(STDIN_FILENO, &ch, 1) > 0)
        {
            int c = std::tolower(ch);
            if (c == 'w')
                v = std::clamp(v + dv, -vmax, vmax);
            else if (c == 's')
                v = std::clamp(v - dv, -vmax, vmax);
            else if (c == 'a')
                w = std::clamp(w + dw, -wmax, wmax);
            else if (c == 'd')
                w = std::clamp(w - dw, -wmax, wmax);
            else if (c == ' ')
            {
                v = 0.0;
                w = 0.0;
            }
            else if (c == 'q')
            {
                quit = true;
                break;
            }
        }

        geometry_msgs::msg::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        pub->publish(msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}