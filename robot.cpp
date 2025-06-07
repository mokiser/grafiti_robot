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
    std::vector<int> pin_handlers; // Хранит файловые дескрипторы для каждого пина

    void gpio_write(int index, uint8_t value) {
        if (index < 0 || index >= pin_handlers.size() || pin_handlers[index] < 0) {
            std::cerr << "Invalid pin index: " << index << std::endl;
            return;
        }
        struct gpiohandle_data data;
        data.values[0] = value;
        if (ioctl(pin_handlers[index], GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
            std::cerr << "Error setting value for pin " << gpio_pins[index] << ": " << strerror(errno) << std::endl;
        }
    }

public:
    GPIOController() : gpio_chip_fd(-1) {
        gpio_chip_fd = open("/dev/gpiochip0", O_RDWR);
        if (gpio_chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO device: " + std::string(strerror(errno)));
        }

        pin_handlers.resize(gpio_pins.size(), -1);
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
            pin_handlers[i] = req.fd;
        }
        stop();
    }

    ~GPIOController() {
        stop();
        for (int handle : pin_handlers) {
            if (handle >= 0) {
                close(handle);
            }
        }
        if (gpio_chip_fd >= 0) {
            close(gpio_chip_fd);
        }
    }

    void forward() {
        gpio_write(2, 1);   // ENA правый
        gpio_write(5, 1);   // ENB левый
        gpio_write(1, 1);   // IN1 правый вперёд
        gpio_write(0, 0);   // IN2 правый реверс
        gpio_write(3, 1);   // IN3 левый вперёд
        gpio_write(4, 0);   // IN4 левый реверс
    }

    void backward() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(1, 0);   // IN1
        gpio_write(0, 1);   // IN2
        gpio_write(3, 0);   // IN3
        gpio_write(4, 1);   // IN4
    }

    void left() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(1, 1);   // IN1
        gpio_write(0, 0);   // IN2
        gpio_write(3, 0);   // IN3
        gpio_write(4, 0);   // IN4
    }

    void right() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(1, 0);   // IN1
        gpio_write(0, 0);   // IN2
        gpio_write(3, 1);   // IN3
        gpio_write(4, 0);   // IN4
    }

    void stop() {
        gpio_write(2, 1);   // ENA
        gpio_write(5, 1);   // ENB
        gpio_write(1, 0);   // IN1
        gpio_write(0, 0);   // IN2
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
        while (true) {
            std::string gpio_data = receiver_.get_gpio_data();
            if (!gpio_data.empty()) {
                std::cout << "Received command: " << gpio_data << std::endl;
                if (gpio_data == "forward") {
                    gpio_.forward();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    gpio_.stop();
                } else if (gpio_data == "left") {
                    gpio_.left();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    gpio_.stop();
                } else if (gpio_data == "right") {
                    gpio_.right();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    gpio_.stop();
                } else if (gpio_data == "backward") {
                    gpio_.backward();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    gpio_.stop();
                } else if (gpio_data == "stop") {
                    gpio_.stop();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    GPIOController& gpio_;
    MqttGpioReceiver& receiver_;
};

int main() {
    try {
        GPIOController gpio;
        MqttGpioReceiver mqtt("192.168.1.103", 1883, "robot/gpio");
        GpioProcessor processor(gpio, mqtt);
        processor.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
