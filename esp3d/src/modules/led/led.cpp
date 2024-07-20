#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

//
#include "../network/netconfig.h"
#include "../usb-serial/usb_serial_service.h"

//
#define NUM_LEDS 1
#define CHIPSET WS2812
Adafruit_NeoPixel g_LedStrip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

struct LedModule
{
    constexpr static auto InterBlinkDelay = 5000;

    LedModule()
    {
        if (Instance == nullptr)
        {
            Instance = this;
            xTaskCreatePinnedToCore(&LedModule::LedTask, "LedTask", 2048, NULL, 1, NULL, 0);
        }
    }
    ~LedModule()
    {
        if (Instance == this)
        {
            Instance = nullptr;
        }
    }

    static void LedTask(void* arg)
    {
        g_LedStrip.begin();
        g_LedStrip.setPixelColor(0, 0);

        while (Instance != nullptr)
        {
            BlinkColor();

            // Delay
            while (Instance->currentDelay > 0)
            {
                constexpr auto DelayStep = 100;

                UpdateMode();

                auto Delay = std::min(Instance->currentDelay, DelayStep);
                vTaskDelay(pdMS_TO_TICKS(Delay));
                Instance->currentDelay -= Delay;
            }
        }

        vTaskDelete(NULL);
    }

    static void UpdateMode()
    {
        auto InitColor = Adafruit_NeoPixel::Color(255, 49, 48);
        auto WifiColor = Adafruit_NeoPixel::Color(100, 52, 235);
        auto UsbColor = Adafruit_NeoPixel::Color(0, 153, 51);

#if defined(USB_SERIAL_FEATURE) && USB_SERIAL_FEATURE
        const bool bIsPrinterConnected = esp3d_usb_serial_service.isConnected();
#else
        const bool bIsPrinterConnected = false;
#endif
        const bool bIsWifiConnected = NetConfig::localIPAddress() != IPAddress((uint32_t)0);

        if (bIsWifiConnected && bIsPrinterConnected)
        {
            Instance->SetColor(UsbColor);
        }
        else if (bIsWifiConnected)
        {
            Instance->SetColor(WifiColor);
        }
        else
        {
            Instance->SetColor(InitColor);
        }
    }

    static void BlinkColor()
    {
        constexpr auto BlinkDuration = 75;

        // Blink once
        ShowColor(Instance->currentColor);
        vTaskDelay(pdMS_TO_TICKS(BlinkDuration));
        ShowColor();

        Instance->currentDelay = InterBlinkDelay - BlinkDuration;
    }

    static void ShowColor(uint32_t color = 0)
    {
        constexpr auto Brightness = 1;
        g_LedStrip.setBrightness(color ? Brightness : 0);

        g_LedStrip.setPixelColor(0, color);
        g_LedStrip.show();
    }

    inline void SetColor(uint32_t color)
    {
        if (currentColor != color)
        {
            currentColor = color;
            currentDelay = 0;
        }
    }

private:
    static LedModule* Instance;

    uint32_t currentColor = 0;
    int currentDelay = 0;

} ledModule;
LedModule *LedModule::Instance = nullptr;