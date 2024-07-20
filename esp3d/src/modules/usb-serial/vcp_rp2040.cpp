#include <array>

#include "usb/vcp.hpp"
#include "../../include/esp3d_config.h"

namespace esp_usb
{

/**
 *  @brief Virtual COM Port Service Class for RP2040
 *
 * Does nothing special, RP2040 chip is supported by default CDC-ACM driver.
 * RP2040 is used for local tests as an emulator of 3d Printer with marlin firmware.
 */
class RP2040CDC : public CdcAcmDevice
{
public:
    /**
     * @brief Constructor for this driver for Rp2040 chip
     *
     * @note USB Host library and CDC-ACM driver must be already installed
     *
     * @param[in] pid            PID eg.
     * @param[in] dev_config     CDC device configuration
     * @param[in] interface_idx  Interface number
     * @return CdcAcmDevice      Pointer to created and opened CH34x device
     */
    RP2040CDC(uint16_t pid, const cdc_acm_host_device_config_t *dev_config, uint8_t interface_idx = 0)
    {
        esp_err_t err = this->open(vid, pid, interface_idx, dev_config);
        if (err != ESP_OK)
        {
            throw (ESP_ERR_NOT_FOUND);
        }
    }

    static constexpr uint16_t vid = 0x2E8A;                     // Raspberry Pi VID
    static constexpr std::array<uint16_t, 1> pids = { 0x000A }; // RP2040 Zero, ANY(0) doesn't work with IDF 5.1, needs update
private:
};

struct DriverRP2040Register
{
    DriverRP2040Register()
    {
        VCP::register_driver<RP2040CDC>();
        esp3d_log_d("RP2040 CDC driver registered");
    }
} RegisterDriverFroRP2040;

} // namespace esp_usb
