#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/core/string_ref.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/json/json_util.h"

#include <array>

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

namespace esphome::emontx {

/// Maximum line length in bytes (plus one byte reserved for null terminator)
static constexpr size_t MAX_LINE_LENGTH = 1024;

/**
 * @class EmonTx
 * @brief Main class for the EmonTx component.
 *
 * The EmonTx processes incoming data frames via UART,
 * extracts tags and values, and publishes them to registered sensors.
 */
class EmonTx final : public Component, public uart::UARTDevice {
 public:
  EmonTx() = default;

  void loop() override;
  void setup() override;
  void dump_config() override;
  void on_shutdown() override;

  template<typename F> void add_on_json_callback(F &&callback) { this->json_callbacks_.add(std::forward<F>(callback)); }

  template<typename F> void add_on_data_callback(F &&callback) { this->data_callbacks_.add(std::forward<F>(callback)); }

  // Send command to emonTx via UART
  void send_command(const std::string &command);

#ifdef USE_SENSOR
  void init_sensors(size_t count) { this->sensors_.init(count); }
  void register_sensor(const char *tag_name, sensor::Sensor *sensor);
#endif

  // Delay after boot before sending 'emonunlock', to let any serial garbage from the
  // reboot settle first. Setting this also locks the emonTx ('emonlock') on shutdown,
  // right before the device reboots (OTA, restart action, safe mode, ...).
  void set_unlock_delay(uint32_t unlock_delay) { this->unlock_delay_ = unlock_delay; }

 protected:
  void parse_json_(const char *data, size_t len);

#ifdef USE_SENSOR
  FixedVector<std::pair<const char *, sensor::Sensor *>> sensors_{};
#endif
  LazyCallbackManager<void(JsonObject, StringRef)> json_callbacks_;
  LazyCallbackManager<void(StringRef)> data_callbacks_;
  uint16_t buffer_pos_{0};
  std::array<char, MAX_LINE_LENGTH + 1> buffer_{};
  uint32_t unlock_delay_{0};
};

// Action to send command to emonTx
template<typename... Ts> class EmonTxSendCommandAction final : public Action<Ts...>, public Parented<EmonTx> {
 public:
  TEMPLATABLE_VALUE(std::string, command)

  void play(const Ts &...x) override { this->parent_->send_command(this->command_.value(x...)); }
};

}  // namespace esphome::emontx
