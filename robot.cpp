#include <iostream>
#include <string>
#include <mosquitto.h>
#include <mutex>
#include <vector>
#include <thread>
#include <cstring>
#include <linux/gpio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

class GPIOController {
private:
    std::vector<int> gpio_pins = {12, 13, 6, 20, 21, 26}; // IN1, IN2, ENA, IN3, IN4, ENB
    int gpio_chip_fd;
    std::vector<int> pin_handles; // Дескрипторы для каждого пина

    void gpio_write(int pin_index, uint8_t value) {
        if (pin_index < 0 || pin_index >= pin_handles.size()) {
            std::cerr << "Invalid pin index: " << pin_index << std::endl;
            return;
        }
        struct gpiohandle_data data;
        data.values[0] = value;
        if (ioctl(pin_handles[pin_index], GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
            std::cerr << "Error setting value for pin " << gpio_pins[pin_index] << ": " << strerror(errno) << std::endl;
        }
    }

public:
    GPIOController() : gpio_chip_fd(-1) {
        gpio_chip_fd = open("/dev/gpiochip0", O_RDWR);
        if (gpio_chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO device: " + std::string(strerror(errno)));
        }

        pin_handles.resize(gpio_pins.size(), -1);
        for (size_t i = 0; i < gpio_pins.size(); ++i) {
            struct gpiohandle_request req;
            memset(&req, 0, sizeof(req));
            req.lineoffsets[0] = gpio_pins[i];
            req.lines = 1;
            req.flags = GPIOHANDLE_REQUEST_OUTPUT;
            req.default_values[0] = 0;
            strcpy(req.consumer_label, "robot_control");
            if (ioctl(gpio_chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
                std::cerr << "Error setting handle for pin " << gpio_pins[i] << ": " << strerror(errno) << std::endl;
                continue;
            }
            pin_handles[i] = req.fd;
        }
        stop();
    }

    ~GPIOController() {
        stop();
        for (int handle : pin_handles) {
            if (handle >= 0) {
                close(handle);
            }
        }
        if (gpio_chip_fd >= 0) {
            close(gpio_chip_fd);
        }
    }

    void forward() {
        gpio_write(2, 1);   // ENA левый включить
        gpio_write(5, 1);   // ENB правый включить
        gpio_write(0, 0);   // IN1 левый вперёд
        gpio_write(1, 1);   // IN2 левый реверс
        gpio_write(3, 1);   // IN3 правый реверс
        gpio_write(4, 0);   // IN4 правый вперёд
    }

    void backward() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(0, 0);   // IN1
        gpio_write(1, 1);   // IN2
        gpio_write(3, 1);   // IN3
        gpio_write(4, 0);   // IN4
    }

    void left() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(0, 0);   // IN1
        gpio_write(1, 0);   // IN2
        gpio_write(3, 1);   // IN3
        gpio_write(4, 0);   // IN4
    }

    void right() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(0, 0);   // IN1
        gpio_write(1, 1);   // IN2
        gpio_write(3, 0);   // IN3
        gpio_write(4, 0);   // IN4
    }

    void stop() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(0, 0);   // IN1
        gpio_write(1, 0);   // IN2
        gpio_write(3, 0);   // IN3
        gpio_write(4, 0);   // IN4
    }
};

class MqttGpioReceiver {
public:
    MqttGpioReceiver(const char* host, int port, const char* topic)
        : mosq_(nullptr), running_(false) {
        mosquitto_lib_init();
        mosq_ = mosquitto_new("robot_client", true, this);
        if (!mosq_) {
            throw std::runtime_error("Failed to create Mosquitto instance");
        }

        mosquitto_message_callback_set(mosq_, message_callback);

        if (mosquitto_connect(mosq_, host, port, 60) != MOSQ_ERR_SUCCESS) {
            mosquitto_destroy(mosq_);
            throw std::runtime_error("Failed to connect to MQTT broker");
        }

        if (mosquitto_subscribe(mosq_, nullptr, topic, 1) != MOSQ_ERR_SUCCESS) {
            mosquitto_destroy(mosq_);
            throw std::runtime_error("Failed to subscribe to topic");
        }

        running_ = true;
        thread_ = std::thread(&MqttGpioReceiver::loop, this);
    }

    ~MqttGpioReceiver() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        if (mosq_) {
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

    std::string get_gpio_data() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!last_gpio_data_.empty()) {
            std::string data = last_gpio_data_;
            last_gpio_data_.clear();
            return data;
        }
        return "";
    }

private:
    static void message_callback(struct mosquitto* mosq, void* obj,
                                const struct mosquitto_message* msg) {
        MqttGpioReceiver* receiver = static_cast<MqttGpioReceiver*>(obj);
        std::lock_guard<std::mutex> lock(receiver->mutex_);
        receiver->last_gpio_data_ = std::string((char*)msg->payload, msg->payloadlen);
        std::cout << "Received command on topic '" << msg->topic << "': " << receiver->last_gpio_data_ << std::endl;
    }

    void loop() {
        while (running_) {
            mosquitto_loop(mosq_, 100, 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    struct mosquitto* mosq_;
    std::string last_gpio_data_;
    std::mutex mutex_;
    std::thread thread_;
    bool running_;
};

class GpioProcessor {
public:
    GpioProcessor(GPIOController& gpio, MqttGpioReceiver& receiver)
        : gpio_(gpio), receiver_(receiver) {}

    void run() {
        std::chrono::steady_clock::time_point last_command_time;
        std::string current_command;

        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::string gpio_data = receiver_.get_gpio_data();

            if (!gpio_data.empty()) {
                current_command = gpio_data;
                last_command_time = now;
            }

            if (current_command.empty() || 
                now - last_command_time > std::chrono::seconds(1)) {
                current_command = "stop";
            }

            if (current_command == "forward") {
                gpio_.forward();
            } else if (current_command == "backward") {
                gpio_.backward();
            } else if (current_command == "left") {
                gpio_.left();
            } else if (current_command == "right") {
                gpio_.right();
            } else {
                gpio_.stop();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    GPIOController& gpio_;
    MqttGpioReceiver& receiver_;
};

int main() {
    try {
        GPIOController gpio;
        MqttGpioReceiver mqtt("192.168.1.100", 1883, "robot/gpio");
        GpioProcessor processor(gpio, mqtt);
        processor.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}