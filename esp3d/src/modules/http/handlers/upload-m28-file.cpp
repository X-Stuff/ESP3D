/*
 upload-m28-files.cpp - ESP3D http handle

 Copyright (c) 2024. All rights reserved.

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with This code; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "../../../include/esp3d_config.h"
#if defined(HTTP_FEATURE) && (COMMUNICATION_PROTOCOL == RAW_SERIAL) && defined(ARDUINO_ARCH_ESP32)
// at the moment only ESP32 is supported
#include <WebServer.h>

#include "../http_server.h"
#include "../../authentication/authentication_service.h"
#include "../../../core/esp3d_settings.h"

//#define m28_log(...) esp3d_log(__VA_ARGS__)
#define m28_log(...) esp3d_log_d(__VA_ARGS__)

#ifdef USB_SERIAL_FEATURE
    #include "../../usb-serial/usb_serial_service.h"
    #define SerialService esp3d_usb_serial_service
#else
    #include "../../serial/serial_service.h"
    #define SerialService esp3d_serial_service
#endif

// Maximum allowed serial transmission speed
#ifdef UPLOAD_THROTTLING_MAX_SPEED
#define UPLOAD_THROTTLING_ENABLED 1
#else
#define UPLOAD_THROTTLING_ENABLED 0
#define UPLOAD_THROTTLING_MAX_SPEED (unsigned int)-1
#endif

#define SERIAL_NEWLINE "\n"

/**
 *  Generic read function that waits for response from serial.
 *
 * MaxSize - size of buffer where response will be stored (you need lesser size for smaller responses)
 *
 * @param response - expected response from serial, if nullptr - will just read everything, during @wait_ms
 * @param out_read - optional output buffer to store read data
 * @param wait_ms  - how long to wait for target response phrase
 *
 * @return true if expected response was received, false otherwise
 */
template<size_t MaxSize = 512> bool wait_serial_response(const char* response, String* out_read = nullptr, uint32_t wait_ms = 1000);

namespace impl
{
/** Explicit implementation for string */
inline size_t send_serial(const String& arg)
{
    return SerialService.writeBytes((const uint8_t*)arg.c_str(), arg.length());
}

/** Explicit implementation for char* */
inline size_t send_serial(const char* arg)
{
    return SerialService.writeBytes((const uint8_t*)arg, strlen(arg));
}

/** Explicit implementation for arbitrary data with size, coupled into tuple */
template<typename TData, typename TNum> inline size_t send_serial(const std::tuple<TData*, TNum>& arg)
{
    return SerialService.writeBytes(reinterpret_cast<const uint8_t*>(std::get<0>(arg)), std::get<1>(arg));
}
}

/** Templated packed version allows to use it like: send_serial("abc", my_string, make_tuple(data, size));  */
template<typename... Args> inline size_t send_serial(Args... args)
{
    // Use a fold expression to call send_serial for each argument
    return (impl::send_serial(args) + ...);
}

/**
 *  Upload update helper structure
 */
struct UploadProgress
{
    UploadProgress(uint32_t threshold_ms = 10000) : report_threshold(threshold_ms) { }

    /** Initalize with filename and total size */
    void init(const String& filename, const uint32_t overall_size);

    /**
     *  Update and calculate stats:
     *
     * @param uploaded_size - how much data was uploaded (sent to UART)
     *
     * @param delta_millis  - delta time that took to send data to UART only
     *                        this needed to calculate real speed of UART transmission
     *                        and throttling application.
     */
    void update(const uint32_t uploaded_size, uint32_t delta_millis);

    /** Make last report */
    void finish();

    /** Return last calculated AVG upload speed in Bytes/s */
    inline uint32_t speed() const { return current_speed; }

private:

    /** Total bytes to be uploaded */
    uint32_t total;

    /** Total uploaded to the moment */
    uint32_t uploaded;

    /** Calculated AVG speed during the update() call */
    uint32_t current_speed;

    /** Timestamp upload started at */
    unsigned long started_at;

    /** Delay between report logs */
    unsigned long report_threshold;

    /** Timestamp of last reported log */
    unsigned long last_reported;

private:
    /** snprintf buffer */
    static char buffer[512];

} g_UploadProgress;

/**
 *  Binary transfer protocol packet.
 * https://github.com/MarlinFirmware/Marlin/blob/74c81117c355612f40061cc07fbad8a619fc5677/Marlin/src/feature/binary_stream.h
 */
union BinaryPacket
{
    //  Should fit USB EP size (USB1.1 is 64 bytes, usually cdc chips implement this standard)
    // You can transfer value up to MAX_CMD_SIZE constant (in marlin firmware) which is 96
    // But because of the way how marlin reads binary packets you will get a lot of 20ms delays
    // which *singificantly* slows down the transfer speed.
    static constexpr uint16_t MAX_PACKET_SIZE = 64;

    // Actual payload size (10 bytes protocol overhead)
    static constexpr uint16_t MAX_DATA_SIZE = MAX_PACKET_SIZE - 10;

    enum class Protocol : uint8_t { CONTROL = 0 << 4, FILE_TRANSFER = 1 << 4, INVALID = 2 << 4 };

    enum class ProtocolControl : uint8_t { SYNC = 1, CLOSE };

    enum class FileTransfer : uint8_t { QUERY, OPEN, CLOSE, WRITE, ABORT };

    static constexpr auto META_SYNC = (uint8_t)Protocol::CONTROL | (uint8_t)ProtocolControl::SYNC;

    static constexpr auto META_CLOSE = (uint8_t)Protocol::CONTROL | (uint8_t)ProtocolControl::CLOSE;

    static constexpr auto META_FILEQUERY = (uint8_t)Protocol::FILE_TRANSFER | (uint8_t)FileTransfer::QUERY;

    static constexpr auto META_FILEOPEN = (uint8_t)Protocol::FILE_TRANSFER | (uint8_t)FileTransfer::OPEN;

    static constexpr auto META_FILEWRITE = (uint8_t)Protocol::FILE_TRANSFER | (uint8_t)FileTransfer::WRITE;

    static constexpr auto META_FILECLOSE = (uint8_t)Protocol::FILE_TRANSFER | (uint8_t)FileTransfer::CLOSE;

    /**
     *  From Marlin firmware:
     *  https://github.com/MarlinFirmware/Marlin/blob/74c81117c355612f40061cc07fbad8a619fc5677/Marlin/src/feature/binary_stream.h#L260
     */
    static uint32_t Checksum(uint32_t cs, uint8_t value)
    {
        uint16_t cs_low = (((cs & 0xFF) + value) % 255);
        return ((((cs >> 8) + cs_low) % 255) << 8)  | cs_low;
    }

    /** Packet header 8 bytes */
    struct
    {
        /**
         * Magic token from Marlin firmware:
         *  https://github.com/MarlinFirmware/Marlin/blob/74c81117c355612f40061cc07fbad8a619fc5677/Marlin/src/feature/binary_stream.h#L212
         */
        uint16_t token = 0xB5AD;

        /** Number of packet in sequeuence */
        uint8_t sync = 0;

        /** protocol and control */
        uint8_t meta = META_SYNC;

        /** Data length (sending part of the file) */
        uint16_t size = 0;

        /** Header checksum (validate e.g. size is correct) */
        uint16_t checksum = 0;

    } header;

    /**
     *  Construct binary packet with specific type and data:
     *
     * @param type   - packet type (CONTROL or FILE_TRANSFER, for control type are also SYNC and CLOSE)
     * @param buffer - data buffer - data to send (if type is FILE_TRANSFER)
     * @param len    - length of data buffer (if type is FILE_TRANSFER)
     * @param sync    - packet number in sequence (if type is FILE_TRANSFER, CONTROL packets usually the only packets)
     */
    BinaryPacket(uint8_t type, const uint8_t* buffer, uint16_t len, uint32_t sync)
    {
        if (len > MAX_DATA_SIZE)
        {
            esp3d_log_e("M28 Binary transfer packet size is too big! (max %d, got %d). Will be truncated!", MAX_DATA_SIZE, len);
        }

        header.token = 0xB5AD;
        header.meta = type;
        header.size = min(len, MAX_DATA_SIZE);
        header.sync = sync % 256;

        // index from where checksum should be calculated
        constexpr static int ChecksumStart = 2;

        // index where header checksum ends
        constexpr static int ChecksumHeaderEnd = 5;

        // index where packet data starts (next byte after header)
        constexpr static int DataStart = sizeof(header);

        const auto length = sizeof(header) + header.size;
        uint16_t checksum = 0;

        for (size_t i = ChecksumStart; i < length; i++)
        {
            if (i >= DataStart)
            {
                data[i] = buffer[i - DataStart];
            }

            checksum = Checksum(checksum, data[i]);

            // Header checksum
            if (i == ChecksumHeaderEnd)
            {
                header.checksum = checksum;
            }
        }

        // Complete checksum of packet
        *(uint16_t*)(&data[length]) = checksum;
    }

    // Close binary stream packet
    static BinaryPacket StreamClose()
    {
        // Unfortunately marlin has some bug or whatever, it at least 1 byte of payload to close the file
        return BinaryPacket(META_CLOSE, nullptr, 0, SyncNumber);
    }

    // Close binary stream packet
    static BinaryPacket StreamInfo()
    {
        // Unfortunately marlin has some bug or whatever, it at least 1 byte of payload to close the file
        return BinaryPacket(META_SYNC, nullptr, 0, SyncNumber);
    }

    // Abort file writing
    static BinaryPacket FileAbort()
    {
        // Unfortunately marlin has some bug or whatever, it at least 1 byte of payload to close the file
        return BinaryPacket((uint8_t)Protocol::FILE_TRANSFER | (uint8_t)FileTransfer::ABORT, nullptr, 0, SyncNumber);
    }

    // Close opened file
    static BinaryPacket FileClose()
    {
        // Unfortunately marlin has some bug or whatever, it at least 1 byte of payload to close the file
        return BinaryPacket(META_FILECLOSE, nullptr, 0, SyncNumber);
    }

    // Open file for writing
    static BinaryPacket FileOpen(const char* filename)
    {
        static uint8_t buffer[MAX_DATA_SIZE];
        buffer[0] = 0; // no dummy
        buffer[1] = 0; // no compression

        // 2 bytes for dummy and compression, 1 byte for null terminator
        constexpr auto MAX_FILE_LEN = MAX_DATA_SIZE - 3;

        // copy filename
        auto filename_len = strlen(filename);
        if (filename_len > MAX_FILE_LEN)
        {
            esp3d_log_e("FileOpen Binary packet failure! Filename: (%s) is too big! (max %d, got %d). Will be truncated!",
                filename, MAX_FILE_LEN, filename_len);

            filename_len = MAX_FILE_LEN;
        }
        memcpy(&buffer[2], filename, filename_len);

        // set null terminator
        const auto buffer_len = 2 + filename_len + 1; // 2 bytes for dummy and compression, 1 byte for null terminator
        buffer[buffer_len - 1] = 0; // null terminator

        return BinaryPacket(META_FILEOPEN, buffer, buffer_len, SyncNumber);
    }

    // Write data to file
    static BinaryPacket FileWrite(const uint8_t* data, size_t len)
    {
        return BinaryPacket(META_FILEWRITE, data, len, SyncNumber);
    }

    /** Full size of packet - how much bytes in `data` has to be sent to serial */
    inline uint16_t Size() const
    {
        // const headere size + data size + checksum
        return sizeof(header) + header.size + sizeof(uint16_t);
    }

    /** Is special type of packet that doesn't affect the packet sequeunce */
    inline bool IsSync() const { return header.meta == META_SYNC; }

    /**
     *  Send own data to printer
     *
     * @param received - if set, here will be placed output read from serial
     * @param specific_response - if you expect something but `okN` - specify it here
     * @param wait_ms - for how long in ms should be serial output read, if 0 will not wait anything
     */
    bool Send(String* received = nullptr, const char* specific_response = nullptr, uint32_t wait_ms = 1000) const
    {
        auto sent = send_serial(std::make_tuple(data, Size()));
        if (sent <= 0)
        {
            if (received != nullptr)
            {
                *received = "FATAL: Serial write failure!";
            }
            return false;
        }

        const bool IsPartofSequence = !IsSync();
        if (IsPartofSequence)
        {
            SyncNumber++;
        }

        if (wait_ms > 0)
        {
            // Wait printer to process the packet
            if (specific_response != nullptr)
            {
                // wait for specific response
                if (!wait_serial_response(specific_response, received, wait_ms))
                {
                    return false;
                }
            }
            else
            {
                char wait_response[8];
                snprintf(wait_response, sizeof(wait_response), "ok%u", header.sync);

                // waiting for specific `ok123` string
                if (!wait_serial_response(wait_response, received, wait_ms))
                {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Data buffer - consist from header, data, last checksum
     */
    uint8_t data[MAX_PACKET_SIZE];

    /** Global sequence number */
    static uint32_t SyncNumber;
};

/** Each sent packet must have consecutive number */
uint32_t BinaryPacket::SyncNumber = 0;

/** Buffer for snprintf output */
char UploadProgress::buffer[512] = {0};

/** Expected file size with (will be written with \r\n) */
static int expected_size = 0;

/** Number of bytes physically transferred over serial to the printer */
static uint32_t uploaded_size = 0;

/** Printer has to be switched to ASCII mode at the end. Otherwise it will just hang forever. */
static bool IsPriterInBinaryMode = false;

/** During upload this bool flag is set and checked in parallel task */
static bool has_any_filewrite_error = false;

/** Upload validation task handle. Task is working only during file transfer, but remains alived until transfer in binary mode */
static TaskHandle_t validation_task_handle = NULL;

/** Asynchrous task to check for file transmission errors (wrong response from printer) */
static void upload_validation_task(void* arg);

/** Consume all data received by serial service */
inline void clear_rx_blocking(int delay = pdMS_TO_TICKS(50))
{
    // clear all late confirmations
    while (SerialService.available() > 0)
    {
        uint8_t buffer[64];
        SerialService.readBytes(buffer, sizeof(buffer));
        vTaskDelay(delay);
    }
}

/**
 *  In order to not try create messages from printer `ok` reponses - we mute usb service
 * However the .readBytes() method will still work, so we can read printer responses if we need it.
 */
inline void mute_usb_serial_service(bool mute)
{
#if defined(USB_SERIAL_FEATURE)
    SerialService.mute(mute);
#endif
}

/** Wrapper helper for reading response and validate task */
inline bool create_upload_validation_task()
{
    if (validation_task_handle == NULL)
    {
        xTaskCreate(&upload_validation_task, "Upld Check", 2048, nullptr, tskIDLE_PRIORITY, &validation_task_handle);
    }

    if (validation_task_handle != NULL)
    {
        vTaskSuspend(validation_task_handle);
    }

    return validation_task_handle != NULL;
}

/**
 *  Sends special binary packet to printer to get current sync number - will be stored in BinaryPacket::SyncNumber
 * @return true if sync number was updated, false otherwise
 */
inline bool update_printer_sync_number()
{
    if (!IsPriterInBinaryMode)
    {
        esp3d_log_e("Can't update sync number. Printer is not in binary mode!");
        return false;
    }

    // Check current state of printer, awainting response like: ss5,96,0.1.0
    // Where ss5 - is a current sync packet number
    // 96 - is a maximum packet size
    // 0.1.0 - is a protocol version
    auto info_packet = BinaryPacket::StreamInfo();
    if (!info_packet.Send(nullptr, nullptr, 0))
    {
        esp3d_log_e("Can't sent binary info packet. Serial write failure");
        return false;
    }

    String info;
    wait_serial_response(nullptr, &info);
    info.trim();

    // Find line that starts with `ss` and take it as a valid response (search from the end)
    auto lastLine = info.lastIndexOf('\n');
    while (lastLine >= 0)
    {
        auto line = info.substring(lastLine + 1);

        if (line.length() > 3 && line.startsWith("ss"))
        {
            info = line;
            break;
        }

        info = info.substring(0, lastLine);
        lastLine = info.lastIndexOf('\n');
    }

    if (info.length() <= 3)
    {
        esp3d_log_e("Can't get printer info. Response is wrong length. Expected at least 3, got: %d", info.length());
        return false;
    }

    if (!info.startsWith("ss"))
    {
        esp3d_log_e("Invalid printer info response, it should start with 'ss'. Response: '%s'", info.c_str());
        return false;
    }

    BinaryPacket::SyncNumber = info.substring(2, info.indexOf(',')).toInt();
    return true;
}

/**
 * Swicth back printer into ASCII mode.
 * If this function fail - printer will be no more repsonsive to anything,
 * and has to be physycally rebooted.
 */
inline bool swicth_back_ascii_mode()
{
    if (IsPriterInBinaryMode)
    {
        // Stream close packet requires correct sync number
        if (!update_printer_sync_number())
        {
            m28_log("Failed to update printer sync number. Switch to ASCII mode most likely fails!");
        }

        String response;
        auto packet = BinaryPacket::StreamClose();
        if (!packet.Send(&response, nullptr))
        {
            esp3d_log_e("Printer didn't respond correctly to switching back to ASCII mode. Response: '%s'", response.c_str());
            return false;
        }

        m28_log("Successfully switched back to ASCII mode.");
        IsPriterInBinaryMode = false;
    }

    return !IsPriterInBinaryMode;
}

/**
 *  Decomposed logic. Upload preparation:
 * - Reset all statistics
 * - Set printer into binary mode with `M28 B1` command
 * - Synchronize with printer's `sync` number
 * - Open file for writing with binary packet
 */
inline bool initialize_upload(const HTTPUpload &upload, WEBSERVER* webserver)
{
    expected_size = 0;
    uploaded_size = 0;

    String filename = upload.filename;
    String sfilename = filename + "S";

    // No / in filename
    if (filename[0] == '/')
    {
        filename.remove(0, 1);
    }

    uint32_t await_upload_size = 0;
    if (webserver->hasArg(sfilename))
    {
        await_upload_size = webserver->arg(sfilename).toInt();
    }
    else if (webserver->hasHeader("Content-Length"))
    {
        await_upload_size = webserver->header("Content-Length").toInt();
    }

    m28_log("Starting upload with M28 B1 command. File: %s (%d bytes)", filename.c_str(), await_upload_size);
    g_UploadProgress.init(filename, await_upload_size);

    // Starting transfer in binary mode
    auto written = send_serial("M28 B1 ", filename, SERIAL_NEWLINE);
    if (written <= 0)
    {
        esp3d_log_e("Can't sent M28 command. Serial write failure");
        return false;
    }
    SerialService.flush();

    String response;
    if (!wait_serial_response("Switching to Binary Protocol", &response))
    {
        esp3d_log_e("Printer didn't respond correctly to M28 command. Response: '%s'", response.c_str());
        return false;
    }

    IsPriterInBinaryMode = true;
    if (!update_printer_sync_number())
    {
        return false;
    }

    String fileopen_response;
    auto file_open = BinaryPacket::FileOpen(filename.c_str());
    if (!file_open.Send(&fileopen_response, "Writing to file"))
    {
        esp3d_log_e("FileOpen binary packet didn't processed correctly. File write didn't start. Response was: '%s'",
            fileopen_response.c_str());
        return false;
    }

    // Before creating task make sure nothing is left for receive
    clear_rx_blocking();

    if (!create_upload_validation_task())
    {
        esp3d_log_e("Cannot create upload validation task! Upload will be aborted!");
        return false;
    }

    has_any_filewrite_error = false;

    m28_log("File opened. Starting upload transmission from sync number: %d", BinaryPacket::SyncNumber);
    return true;
}

/**
 *  Decomposed logic. Upload processing:
 * - Sends file content to printer
 * - Update send statistics (calculate speed, throttle if necessary)
 */
inline bool process_upload(const HTTPUpload &upload)
{
    if (validation_task_handle == NULL)
    {
        esp3d_log_e("Can't upload file content. Validation task not running!");
        return false;
    }

    if (!IsPriterInBinaryMode)
    {
        esp3d_log_e("Can't upload file content. Printer is not in binary mode!");
        return false;
    }

    // Use later as instant upload speed calculation
    auto upload_start_time = millis();

    const uint8_t *file = upload.buf;
    const size_t size = upload.currentSize;

    vTaskResume(validation_task_handle);

    uint16_t block_size = 0;
    for(size_t i = 0; i < size && !has_any_filewrite_error; i += block_size)
    {
        vTaskDelay(1);

        block_size = (uint16_t)min(size - i, (size_t)BinaryPacket::MAX_DATA_SIZE);

        // We're sending data to printer without blocking checks for each packet
        // The parallel task will do this constantly in background loop
        auto packet = BinaryPacket::FileWrite(&file[i], block_size);
        if (!packet.Send(nullptr, nullptr, 0))
        {
            esp3d_log_e("FileWrite Packet #%d wasn't sent! Serial transmission failure!", BinaryPacket::SyncNumber);
            has_any_filewrite_error = true;
            break;
        }
        expected_size += block_size;
        uploaded_size += packet.Size();
    }

    // No need to update throttling or statistics - transfer is failed, just return, validation task will suspend self
    if (has_any_filewrite_error)
    {
        return false;
    }

    // if `has_any_filewrite_error` is true - task will suspend self gracefully
    vTaskSuspend(validation_task_handle);

    if constexpr (UPLOAD_THROTTLING_ENABLED)
    {
        // Maximum real CDC communication speed is 480Kbit/s;
        // Throttling is necessary, otherwise data will be just lost
        // The problem is: the correct way is ask from receiver side to delay,
        // but *CURRENT* UART implementation doesn't support this (CTS/RTS unavalable)
        constexpr auto maxspeed = (UPLOAD_THROTTLING_MAX_SPEED / 8);

        const auto real_upload_time_ms = millis() - upload_start_time;
        auto instant_speed = real_upload_time_ms > 0 ? (upload.currentSize * 1000) / real_upload_time_ms : 0;

        if (instant_speed > maxspeed)
        {
            auto max_allowed_bytes = (real_upload_time_ms * maxspeed) / 1000;
            auto overflow_bytes = upload.currentSize - max_allowed_bytes;

            static uint32_t cumulative_delay = 0;

            auto instant_delay_ms = cumulative_delay + max(1, (int)(1000 * overflow_bytes / maxspeed));
            if (instant_delay_ms >= 5)
            {
                cumulative_delay = 0;

                m28_log("Upload speed: (%d KB/s) is over the threshold: (%d KB/s). Initiate throttling! Delay: %d ms",
                     instant_speed / 1024, maxspeed / 1024, instant_delay_ms);

                ESP3DHal::wait(instant_delay_ms);
            }
            else
            {
                cumulative_delay += instant_delay_ms;
            }
        }
    }

    const auto throttled_upload_time = millis() - upload_start_time;
    g_UploadProgress.update(upload.currentSize, throttled_upload_time);

    return true;
}

/**
 *  Decomposed logic. Upload finalization on success of failrue
 * - Close file on printer
 * - Switch printer back to ASCII mode
 * - Finalize upload statistics
 */
inline bool finalize_upload(const HTTPUpload &upload, String& error)
{
    clear_rx_blocking();

    // Close file on printer
    String file_close_response;
    auto file_close = BinaryPacket::FileClose();
    if (!file_close.Send(&file_close_response, "PFT:success", 3000))
    {
        error = "FileClose binary packet failure: Reponse: '" + file_close_response + "'";
        return false;
    }

    // Switch back to ASCII mode
    if (!swicth_back_ascii_mode())
    {
        error = "FATAL: Can't switch back to ASCII mode. Printer MUST be rebooted!";
        return false;
    }

    // ReInit SD (required after binary mode)
    if (!send_serial("M21" SERIAL_NEWLINE))
    {
        error = "FATAL: Can't send M21 command to initialize SD card! Gerenal serial write failure!";
        return false;
    }

    String init_sd_response;
    if (!wait_serial_response("SD card ok", &init_sd_response))
    {
        error = "SD card was not initialized! Response: '" + init_sd_response + "'";
        return false;
    }

    return true;
}

/**
 *  Post upload checks:
 * - ask printer about files in its SD (M20)
 * - find uploaded file by name and check if size is correct
 */
inline bool check_file_size(const int expected, const String& filename)
{
    m28_log("Validate file size on SD card. Expected: %u bytes", expected);

    if (!send_serial("M20" SERIAL_NEWLINE))
    {
        esp3d_log_e("Check file size failed! Writing to serial failed!");
        return false;
    }
    SerialService.flush();

    String list_files_response;
    if (!wait_serial_response("End file list", &list_files_response))
    {
        esp3d_log_e("M20 List files command failed! Cannot detect if %s file exists!. Response was: '%s'",
            filename.c_str(), list_files_response.c_str());
        return false;
    }

    auto line_start = 0;
    auto line_end = list_files_response.indexOf("\n", line_start);

    auto filename_upper = filename;
    filename_upper.toUpperCase();
    auto filename_8d3 = filename_upper.substring(0, 5) + "~1.";

    while (line_end >= 0 && line_end < (int)list_files_response.length())
    {
        auto line = list_files_response.substring(line_start, line_end);
        line.trim();
        line.toUpperCase();

        // Example of line:
        // FAT16/32: MYFILE.GCO; Size: 14129
        //
        // MYFILE.GCO 14129
        // OTHER~1.GCO 5489

        if (line.indexOf(filename_upper) >= 0 || line.indexOf(filename_8d3) >= 0)
        {
            auto filesize_starts = line.lastIndexOf(" ") + 1; // +1 and thus we have `> 0` and not `>= 0`
            if (filesize_starts > 0 && filesize_starts < (int)line.length())
            {
                auto filesize_str = line.substring(filesize_starts); //Will be: 14129
                auto filesize = filesize_str.toInt();

                if (filesize == expected)
                {
                    m28_log("File check complete! File size on SD (%u Bytes) match expected size: %u", filesize, expected_size);
                    return true;
                }

                esp3d_log_e("M20 List files command failed! Files size doesn't match! Expected: %d, SD Size:%d ('%s'). Parsed from line: '%s' ",
                    expected, filesize, filesize_str.c_str(), line.c_str());
                return false;
            }
            else
            {
                esp3d_log_e("M20 List files command failed! Cannot parse file size from line: %s!", line.c_str());
                return false;
            }
        }

        line_start = line_end + 1;
        if (line_start >= (int)list_files_response.length())
        {
            break;
        }

        line_end = list_files_response.indexOf("\n", line_start);
    }

    esp3d_log_e("M20 List files command failed! Cannot find file: %s (%s) in response:" SERIAL_NEWLINE "%s",
        filename_upper.c_str(), filename_8d3.c_str(), list_files_response.c_str());
    return false;
}

/**
 * Delete uploaded file from printer's SD card
 */
inline bool delete_uploaded_file(const String& filename)
{
    m28_log("Deleting file: '%s' size on SD card", filename.c_str());

    if (!send_serial("M30 /", filename, SERIAL_NEWLINE))
    {
        esp3d_log_e("M30 Delete file command failed! Writing to serial failed!");
        return false;
    }

    String response;
    if (!wait_serial_response("File deleted:", &response))
    {
        esp3d_log_e("M30 Delete file command failed! Cannot delete file: '%s'!. Response was: '%s'",
            filename.c_str(), response.c_str());
        return false;
    }

    return true;
}

/**
 *  When upload is failed (printer response has some abnormalities) in the middle of transmission - abort it.
 * - Send file close packet
 * - Switch printer back to ASCII mode
 * - Delete uploaded file from printer's SD card
 *
 * @return error message if something went wrong, empty string otherwise
 */
inline String abort_upload(const HTTPUpload &upload)
{
    String error;
    if (finalize_upload(upload, error))
    {
        if (!delete_uploaded_file(upload.filename))
        {
            error = "Can't delete uploaded file: " + upload.filename + ". Probably it doesn't fits the 8 dot 3 format.";
        }
    }
    return error;
}

// M28 files uploader handle
void HTTP_Server::M28Fileupload()
{
    // get authentication status
    ESP3DAuthenticationLevel auth_level = AuthenticationService::getAuthenticatedLevel();

    // Guest cannot upload - only admin
    if (auth_level == ESP3DAuthenticationLevel::guest)
    {
        pushError(ESP_ERROR_AUTHENTICATION, "Upload rejected", 401);
        _upload_status = UPLOAD_STATUS_FAILED;
        cancelUpload();
        return;
    }

    HTTPUpload& upload = _webserver->upload();
    if (upload.status == UPLOAD_FILE_START)
    {
        if (initialize_upload(upload, _webserver))
        {
            _upload_status = UPLOAD_STATUS_ONGOING;
            mute_usb_serial_service(true);
        }
        else
        {
            _upload_status = UPLOAD_STATUS_FAILED;
            HTTP_Server::pushError(ESP_ERROR_FILE_WRITE, "Failed to initialize upload!");
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
        if (_upload_status == UPLOAD_STATUS_ONGOING)
        {
            if (!process_upload(upload))
            {
                _upload_status = UPLOAD_STATUS_FAILED;
                HTTP_Server::pushError(ESP_ERROR_UPLOAD, "Failed to process uploaded block!");

                esp3d_log_e("Upload failed! After transferring %u bytes (%u byte of file). Last packet sync: %u. Aborting...",
                    uploaded_size, expected_size, BinaryPacket::SyncNumber);

                String error = abort_upload(upload);
                if (!error.isEmpty())
                {
                    esp3d_log_e("Upload abortion failed! Error: %s", error.c_str());
                }
            }
        }
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
        if (_upload_status == UPLOAD_STATUS_ONGOING)
        {
            String error;
            if (finalize_upload(upload, error))
            {
                g_UploadProgress.finish();
                m28_log("File upload Complete! Closing file... Expected file size on SD: %u bytes. Total TX bytes: %u",
                    expected_size, uploaded_size);

                _upload_status = UPLOAD_STATUS_SUCCESSFUL;
            }
            else
            {
                _upload_status = UPLOAD_STATUS_FAILED;

                esp3d_log_e("Finalize Upload FAIL: %s", error.c_str());
                HTTP_Server::pushError(ESP_ERROR_FILE_CLOSE, error.c_str());
            }
        }
    }
    else
    {
        // error
        _upload_status = UPLOAD_STATUS_FAILED;
        HTTP_Server::pushError(ESP_ERROR_FILE_WRITE, "File write aborted");
        esp3d_log_e("File upload aborted or there was a network failure!");
    }

    if (_upload_status == UPLOAD_STATUS_FAILED)
    {
        cancelUpload();
    }

    if (_upload_status == UPLOAD_STATUS_SUCCESSFUL)
    {
        // filepath without leading / (since M20 command omits it)
        auto filepath = upload.filename.startsWith("/") ? upload.filename.substring(1) : upload.filename;
        if (!check_file_size(expected_size, filepath))
        {
            HTTP_Server::pushError(ESP_ERROR_FILE_WRITE, "Uploaded file corrupted!");
            if (!delete_uploaded_file(filepath))
            {
                HTTP_Server::pushError(ESP_ERROR_FILE_WRITE, "Can't delete uploaded file!");
            }
        }
        else
        {
            m28_log("File uploaded successfully: %s", filepath.c_str());
        }
    }

    // if upload is finished/cancelled/aborted
    if (_upload_status != UPLOAD_STATUS_ONGOING)
    {
        if (validation_task_handle)
        {
            if (has_any_filewrite_error)
            {
                // Just wait a little, the task might need to print somthing if case of error
                ESP3DHal::wait(100);
            }

            vTaskDelete(validation_task_handle);
            validation_task_handle = NULL;
        }

        if (!swicth_back_ascii_mode())
        {
            HTTP_Server::pushError(ESP_ERROR_FILE_WRITE, "Printer still in binary mode. Reboot required!");
        }

        // we should unmute usb serial service to allow message forwarding, but wait a little for last messages from printer
        ESP3DHal::wait(100);
        clear_rx_blocking();
        mute_usb_serial_service(false);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UploadProgress::init(const String& filename, const uint32_t overall_size)
{
    total = overall_size;
    current_speed = last_reported = uploaded = 0;
    started_at = millis();
}

void UploadProgress::update(const uint32_t uploaded_size, uint32_t delta_millis)
{
    uploaded += uploaded_size;
    bool finished = uploaded == total;

    if (delta_millis > 0)
    {
        current_speed = (uploaded_size * 1000 / delta_millis);
    }

    if (millis() - last_reported > report_threshold)
    {
        const auto upload_pct = finished ? 100 : (int)ceil((uploaded / (float)total) * 100);

        const unsigned long total_elapsed_ms = (millis() - started_at);
        const unsigned long total_elapsed = total_elapsed_ms / 1000;

        unsigned long rest = total - uploaded;
        const auto ETA = finished ? total_elapsed : (current_speed > 0 ? rest / current_speed : 0);
        const auto ETAs = ETA % 60;
        const auto ETAm = ETA / 60;

        { // message allocation range

            const auto speed = (finished ? (total / total_elapsed_ms) * 1000 : current_speed) / 1024;
            snprintf(UploadProgress::buffer, sizeof(UploadProgress::buffer),
                "%s: %dKb (%d%%), %s speed: %ldKb/s, %s: %.2ldm %.2lds",
                (finished ? "Finished" : "Uploaded"),
                uploaded / 1024, upload_pct,
                (finished ? "Average" : "Write"), speed,
                (finished ? "Elapsed" : "ETA"),
                ETAm, ETAs);

            m28_log("%s", UploadProgress::buffer);
            last_reported = millis();
        }
    }
}

void UploadProgress::finish()
{
    last_reported = 0;
    uploaded = total;

    // Last report
    update(0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** */
inline bool is_valid_response_char(char ch)
{
    if (IsPriterInBinaryMode)
    {
        return ch == 'o' || ch == 'k' || isdigit(ch) || isspace(ch);
    }
    return true;
}

/** */
void upload_validation_task(void* arg)
{
    static uint8_t response_buffer[128];
    do
    {
        bool is_response_invalid = false;

        auto num_read = SerialService.readBytes(response_buffer, sizeof(response_buffer));
        auto i = 0;
        for (;i < num_read; ++i)
        {
            // valid responses are only `okN` - where N is a number and newlines
            if (!is_valid_response_char(response_buffer[i]))
            {
                is_response_invalid = true;
                break;
            }
        }

        // Error might happen ouside task - it has to be suspended as well
        if (has_any_filewrite_error || is_response_invalid)
        {
            if (is_response_invalid)
            {
                while (auto zero = memchr(response_buffer, 0, sizeof(response_buffer)))
                {
                    *(char*)zero = 0x01;
                }

                esp3d_log_e("Upload validation task detected transmission error! Invalid character at index: %d - '%c' (0x%x) in [len:%d, data:'%.*s']",
                    i, response_buffer[i], response_buffer[i], num_read, num_read, response_buffer);

                has_any_filewrite_error = true;
            }
            vTaskSuspend(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    while (true);
}

/**
 *  Generic read function that waits for response from serial.
 *
 * MaxSize - size of buffer where response will be stored (you need lesser size for smaller responses)
 *
 * @param target_str - expected response from serial, if nullptr - will just read everything, during @wait_ms
 * @param out_read - optional output buffer to store read data, if target_str is nullptr - will store everything
 * @param wait_ms  - how long to wait for target response phrase
 *
 * @return true if expected response was received, false otherwise
 */
template<size_t MaxSize> bool wait_serial_response(const char* target_str, String* out_read, uint32_t wait_ms)
{
    // length of response to wait for (if response is nullptr - we just wait for any data)
    auto awaiting_len = target_str ? strlen(target_str) : MaxSize - 1;

    static char response_buffer[MaxSize];
    auto response_read = 0;
    bool response_found = false;

    if (awaiting_len >= sizeof(response_buffer))
    {
        esp3d_log_e("Response to wait for is too big! (need %d, has %d)", awaiting_len, sizeof(response_buffer));
        return false;
    }

    TickType_t timeout_ticks = pdMS_TO_TICKS(wait_ms);
    TimeOut_t connection_timeout;
    vTaskSetTimeOutState(&connection_timeout);
    do
    {
        auto available = sizeof(response_buffer) - response_read;
        if (available <= 0)
        {
            // buffer is full
            esp3d_log_e("Response buffer is full! Rewind! Data already in buffer: '%.*s'.", sizeof(response_buffer), response_buffer);
            response_read = 0;
            available = sizeof(response_buffer);
        }

        // append to previous read
        uint8_t* line = (uint8_t*)&response_buffer[response_read];

        auto read = SerialService.readBytes(line, available);
        if (read <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        response_read += read;

        // re-set timeout after each read (if we receive a lot of data)
        vTaskSetTimeOutState(&connection_timeout);

        // Check every received line for response for early exit
        if (response_read >= awaiting_len)
        {
            // But only if we have something to wait for
            if (target_str != nullptr)
            {
                response_found = strstr(response_buffer, target_str) != nullptr;
            }
        }
    }
    while (!response_found && xTaskCheckForTimeOut(&connection_timeout, &timeout_ticks) == pdFALSE);

    if (out_read != nullptr)
    {
        // Replace \0 with spaces
        while(auto zero = memchr(response_buffer, 0, response_read))
        {
            *(char*)zero = ' ';
        }

        *out_read = String(response_buffer, response_read);
    }

    return response_found;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // HTTP_FEATURE && (COMMUNICATION_PROTOCOL == RAW_SERIAL) && defined(ARDUINO_ARCH_ESP32)
