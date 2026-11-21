#include "CenturyVSPump.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome
{
    namespace century_vs_pump
    {

        //////////////////////////////////////////////////////////////////////////////////////////////
        //
        //  CenturyVSPump implementation
        //
        /////////////////////////////////////////////////////////////////////////////////////////////

        static const char *const TAG = "century_vs_pump";

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::setup()
        {
#ifdef MODBUS_ENABLE_SWITCH
            enabled_switch_ = new CenturyPumpEnabledSwitch();
            enabled_switch_->set_name(name_ + " MODBUS enabled");
            App.register_switch(enabled_switch_);
#endif
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::loop()
        {
            // Incoming data to process?
            if (!response_queue_.empty())
            {
                auto &message = response_queue_.front();
                if (message != nullptr)
                    process_modbus_data_(message.get());
                response_queue_.pop();
            }
            else
            {
                // all messages processed send pending commmands
                send_next_command_();
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::update()
        {
            // Request status & pump RPM
#ifdef MODBUS_ENABLE_SWITCH
            if (enabled_switch_ == nullptr)
                return;
            if (enabled_switch_->state == 0)
                return;
#endif
            ESP_LOGV(TAG, "Updating pump component");
            for (auto item : items_)
                queue_command_(item->create_command());
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// called when a modbus response was parsed without errors
        void CenturyVSPump::on_response(std::span<const uint8_t> request_pdu, std::span<const uint8_t> response_pdu)
        {
            ESP_LOGV(TAG, "Pump got data");
            if (!in_flight_ || command_queue_.empty())
                return;
            auto &current_command = this->command_queue_.front();
            if (response_pdu.empty() || response_pdu[0] != current_command->function_)
            {
                ESP_LOGW(TAG, "Unexpected response, ignoring");
                return;
            }
            // Response PDU is [function][ACK][data...], keep everything after the function code
            current_command->payload_.assign(response_pdu.begin() + 1, response_pdu.end());
            this->response_queue_.push(std::move(current_command));
            ESP_LOGV(TAG, "Pump response queued");
            command_queue_.pop_front();
            in_flight_ = false;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// called when a modbus error response was received
        void CenturyVSPump::on_error(std::span<const uint8_t> request_pdu, ExceptionCode exception_code)
        {
            ESP_LOGV(TAG, "Received modbus error");
            if (!in_flight_ || command_queue_.empty())
                return;
            ESP_LOGD(TAG, "Modbus error %02X, so removing current command (%02X) from queue", (uint8_t)exception_code,
                     command_queue_.front()->function_);
            command_queue_.pop_front();
            in_flight_ = false;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// called when no response arrived in time
        bool CenturyVSPump::on_no_response(std::span<const uint8_t> request_pdu)
        {
            if (!in_flight_ || command_queue_.empty())
                return false;
            auto &command = command_queue_.front();
            if (--command->send_countdown > 0)
            {
                ESP_LOGV(TAG, "No response to command %02X, retrying", command->function_);
                return true;
            }
            ESP_LOGD(TAG, "Pump command %02X no response received - removed from send queue", command->function_);
            command_queue_.pop_front();
            in_flight_ = false;
            return false;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        /// called when the hub dropped our request before sending it
        void CenturyVSPump::on_not_sent(std::span<const uint8_t> request_pdu)
        {
            if (!in_flight_ || command_queue_.empty())
                return;
            ESP_LOGD(TAG, "Pump command %02X not sent - removed from send queue", command_queue_.front()->function_);
            command_queue_.pop_front();
            in_flight_ = false;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::dump_config()
        {
            ESP_LOGCONFIG(TAG, "CenturyVSPump:");
            ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::queue_command_(const CenturyPumpCommand &command)
        {
#ifdef MODBUS_ENABLE_SWITCH
            if (enabled_switch_ == nullptr)
                return;
            if (enabled_switch_->state == 0)
                return;
#endif
            command_queue_.push_back(std::make_unique<CenturyPumpCommand>(command));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        void CenturyVSPump::process_modbus_data_(const CenturyPumpCommand *response)
        {
            if (response->payload_.empty())
            {
                ESP_LOGW(TAG, "Empty MODBUS response payload, ignoring");
                return;
            }

            // payload_ is [ACK][data...], ensure ACK was OK
            if (response->payload_[0] != 0x10)
            {
                ESP_LOGW(TAG, "Function %X NACK with %X, ignoring", response->function_, response->payload_[0]);
                return;
            }

            // Pass remaining payload to handler
            std::vector<uint8_t> data(response->payload_.begin() + 1, response->payload_.end());

            response->on_data_func_(this, data);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        bool CenturyVSPump::send_next_command_()
        {
            uint32_t last_send = millis() - this->last_command_timestamp_;
            if ((last_send > this->command_throttle_) && !in_flight_ && !command_queue_.empty())
            {
                auto &command = command_queue_.front();

                ESP_LOGV(TAG, "Sending command with function %02X", command->function_);
                if (command->send())
                {
                    // The hub now owns the transaction, retries are driven by on_no_response()
                    in_flight_ = true;
                }
                else
                {
                    ESP_LOGW(TAG, "Pump command %02X refused by modbus hub - removed from send queue", command->function_);
                    command_queue_.pop_front();
                }
                this->last_command_timestamp_ = millis();
            }
            return true;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        bool CenturyPumpCommand::send()
        {
            // PDU is [function][ACK][payload...], the hub prepends the device address and appends the CRC
            std::vector<uint8_t> pdu;
            pdu.push_back(function_);
            pdu.push_back(0x20);
            pdu.insert(pdu.end(), payload_.begin(), payload_.end());
            return pump_->queue_pdu(pdu);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_status_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump, bool running)> on_status_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x43; // Pump status
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                if (data.empty())
                    return;
                ESP_LOGD(TAG, "Got status command reply %02X", data[0]);

                if (data[0] == 0x00)
                    on_status_func(pump, false);
                else if (data[0] == 0x0B)
                    on_status_func(pump, true);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_read_sensor_command(CenturyVSPump *pump, uint8_t page, uint8_t address, uint16_t scale, std::function<void(CenturyVSPump *pump, uint16_t value)> on_value_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x45; // Read sensor
            cmd.payload_.push_back(page);
            cmd.payload_.push_back(address);
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                // data is [page][address][value lo][value hi?]
                if (data.size() < 3)
                    return;
                // Always going to have at least 1 byte of sensor data
                uint16_t value = (uint16_t)data[2];
                if (data.size() == 4)
                {
                    // But sometimes, we get two bytes
                    value |= (uint16_t)data[3] << 8;
                }
                // Scale the value
                value /= scale;
                ESP_LOGD(TAG, "Read value %d from page %d, addr %d", value, page, address);
                on_value_func(pump, value);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_run_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump)> on_confirmation_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x41; // Go
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Confirmed pump running");
                on_confirmation_func(pump);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_stop_command(CenturyVSPump *pump, std::function<void(CenturyVSPump *pump)> on_confirmation_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x42; // Stop
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Confirmed pump stopped");
                on_confirmation_func(pump);
            };
            return cmd;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        CenturyPumpCommand CenturyPumpCommand::create_set_demand_command(CenturyVSPump *pump, uint16_t demand, std::function<void(CenturyVSPump *pump)> on_confirmation_func)
        {
            CenturyPumpCommand cmd = {};
            cmd.pump_ = pump;
            cmd.function_ = 0x44;      // Set demand
            cmd.payload_.push_back(0); // Mode (0=Speed, 1=Torque, 2=Reserved, 3=Reserved)
            demand *= 4;               // Scaling
            cmd.payload_.push_back(demand & 0xff);
            cmd.payload_.push_back(demand >> 8);
            cmd.on_data_func_ = [=](CenturyVSPump *pump, const std::vector<uint8_t> data)
            {
                ESP_LOGD(TAG, "Set demand comfirmed");
                on_confirmation_func(pump);
            };
            return cmd;
        }
    }
}