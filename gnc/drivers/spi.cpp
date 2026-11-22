/*
 * NVIDEA JETSON ORIN NANO SPI DRIVER
 * Author: Arman Islam
 * g++ -std=c++17 -O2 -Wall -Wextra -pthread spi.cpp -o spi
 * ./spi SPI_DEVICE GPIO_CHIP GPIO_LINE_OFFSET
 *
 *
 *
 *
 *
 *
 *
 */

#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <string_view>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr uint32_t spi_speed_hz    = 1000000;
constexpr uint8_t spi_read         = 0x80;
constexpr uint8_t sample_rate_div  = 0x19;
constexpr uint8_t config           = 0x1a;
constexpr uint8_t gyro_config      = 0x1b;
constexpr uint8_t accel_config     = 0x1c;
constexpr uint8_t int_pin_config   = 0x37;
constexpr uint8_t int_enable       = 0x38;
constexpr uint8_t int_status       = 0x3a;
constexpr uint8_t accel_xout_high  = 0x3b;
constexpr uint8_t user_control     = 0x6a;
constexpr uint8_t power_management = 0x6b;
constexpr uint8_t who_am_i         = 0x75;

bool transfer(int fd, const uint8_t* tx, uint8_t* rx, uint32_t length) {
    spi_ioc_transfer message{};
    message.tx_buf        = reinterpret_cast<std::uintptr_t>(tx);
    message.rx_buf        = reinterpret_cast<std::uintptr_t>(rx);
    message.len           = length;
    message.speed_hz      = spi_speed_hz;
    message.bits_per_word = 8;

    const int result = ioctl(fd, SPI_IOC_MESSAGE(1), &message);
    if (result < 0) {
        std::perror("MPU-6000 SPI transfer");
        return false;
    }
    if (static_cast<uint32_t>(result) != length) {
        std::cerr << "Incomplete MPU-6000 SPI transfer\n";
        return false;
    }
    return true;
}

bool write_register(int fd, uint8_t reg, uint8_t value) {
    const uint8_t tx[] = {reg, value};
    return transfer(fd, tx, nullptr, sizeof(tx));
}

bool read_register(int fd, uint8_t reg, uint8_t& value) {
    const uint8_t tx[] = {static_cast<uint8_t>(reg | spi_read), 0};
    uint8_t rx[sizeof(tx)]{};
    if (!transfer(fd, tx, rx, sizeof(tx))) {
        return false;
    }
    value = rx[1];
    return true;
}

bool initialize(int fd) {
    using namespace std::chrono_literals;
    uint8_t mode   = SPI_MODE_0;
    uint8_t bits   = 8;
    uint32_t speed = spi_speed_hz;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::perror("Configure SPI");
        return false;
    }

    uint8_t identity = 0;
    if (!read_register(fd, who_am_i, identity)) {
        return false;
    }
    if (identity != 0x68) {
        std::cerr << "Expected MPU-6000 WHO_AM_I 0x68, got 0x" << std::hex
                  << static_cast<unsigned>(identity) << '\n';
        return false;
    }

    if (!write_register(fd, power_management, 0x80)) { // Device reset
        return false;
    }
    std::this_thread::sleep_for(100ms);
    if (!write_register(fd, user_control, 0x10) ||     // Disable primary I2C
        !write_register(fd, power_management, 0x01)) { // Wake with X gyro PLL
        return false;
    }
    std::this_thread::sleep_for(100ms);

    // DLPF enables a 1 kHz base rate. Divide by 10 for 100 Hz samples.
    return write_register(fd, config, 0x03) && write_register(fd, sample_rate_div, 9) &&
           write_register(fd, gyro_config, 0x00) &&  // +/-250 degrees/s
           write_register(fd, accel_config, 0x00) && // +/-2 g
           // Active high, push-pull, latched, cleared by a register read.
           write_register(fd, int_pin_config, 0x30);
}

int request_interrupt(const char* chip_path, uint32_t offset) {
    const int chip_fd = open(chip_path, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) {
        std::perror(chip_path);
        return -1;
    }

    gpioevent_request request{};
    request.lineoffset  = offset;
    request.handleflags = GPIOHANDLE_REQUEST_INPUT;
    request.eventflags  = GPIOEVENT_REQUEST_RISING_EDGE;
    std::snprintf(request.consumer_label, sizeof(request.consumer_label), "mpu6000");
    const int result = ioctl(chip_fd, GPIO_GET_LINEEVENT_IOCTL, &request);
    if (result < 0) {
        std::perror("Request MPU-6000 interrupt GPIO");
    }
    close(chip_fd);
    return result < 0 ? -1 : request.fd;
}

int stream_samples(int spi_fd, int irq_fd) {
    // Clear pending status before enabling the interrupt with the GPIO armed.
    uint8_t status = 0;
    if (!read_register(spi_fd, int_status, status) || !write_register(spi_fd, int_enable, 0x01)) {
        return 1;
    }

    const std::array<uint8_t, 15> tx{accel_xout_high | spi_read};
    std::array<uint8_t, tx.size()> rx{};
    std::cout << "irq_timestamp_ns,accel_x,accel_y,accel_z,temp_raw,gyro_x,gyro_y,gyro_z\n";
    while (true) {
        gpioevent_data event{};
        const ssize_t count = read(irq_fd, &event, sizeof(event));
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::perror("Wait for MPU-6000 interrupt");
            return 1;
        }
        if (count != sizeof(event)) {
            std::cerr << "Incomplete GPIO edge event\n";
            return 1;
        }

        // The burst also clears the interrupt latch for the next rising edge.
        if (!transfer(spi_fd, tx.data(), rx.data(), rx.size())) {
            return 1;
        }

        std::cout << event.timestamp;
        for (std::size_t i = 1; i < rx.size(); i += 2) {
            const int word   = (rx[i] << 8) | rx[i + 1];
            const int sample = word >= 0x8000 ? word - 0x10000 : word;
            std::cout << ',' << sample;
        }
        std::cout << std::endl;
        if (!std::cout) {
            return 1;
        }
    }
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " SPI_DEVICE GPIO_CHIP GPIO_LINE_OFFSET\n";
        return 1;
    }

    const std::string_view offset_arg = argv[3];
    uint32_t offset                   = 0;
    const auto result =
        std::from_chars(offset_arg.data(), offset_arg.data() + offset_arg.size(), offset);
    if (result.ec != std::errc{} || result.ptr != offset_arg.data() + offset_arg.size()) {
        std::cerr << "GPIO_LINE_OFFSET must be an unsigned decimal line offset.\n";
        return 1;
    }

    const int spi_fd = open(argv[1], O_RDWR | O_CLOEXEC);
    if (spi_fd < 0) {
        std::perror(argv[1]);
        return 1;
    }
    int exit_status = 1;
    if (initialize(spi_fd)) {
        const int irq_fd = request_interrupt(argv[2], offset);
        if (irq_fd >= 0) {
            exit_status = stream_samples(spi_fd, irq_fd);
            write_register(spi_fd, int_enable, 0);
            close(irq_fd);
        }
    }
    close(spi_fd);
    return exit_status;
}
