#include "emontx_ha_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome::emontx_ha_bridge {

static const char *const TAG = "emontx_ha_bridge";

void EmonTxHaBridge::setup() {
  // Auto-register the send_command service so HA can configure the emonTx hardware
  this->register_service(&EmonTxHaBridge::on_send_command_, "send_command", {"command"});

  // Fire esphome.emontx_raw for every raw serial line received
  this->emontx_->add_on_data_callback([this](StringRef line) {
    if (!this->is_connected())
      return;
    this->fire_homeassistant_event("esphome.emontx_raw", {
        {"device_id", App.get_name().str()},
        {"line", line.str()},
    });
  });

  // Fire esphome.emontx_json for every successfully parsed JSON frame
  this->emontx_->add_on_json_callback([this](JsonObject /*json*/, StringRef raw_json) {
    if (!this->is_connected())
      return;
    this->fire_homeassistant_event("esphome.emontx_json", {
        {"device_id", App.get_name().str()},
        {"data", raw_json.str()},
    });
  });
}

void EmonTxHaBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "EmonTx HA Bridge:");
  ESP_LOGCONFIG(TAG, "  HA events : esphome.emontx_raw, esphome.emontx_json");
  ESP_LOGCONFIG(TAG, "  HA service: send_command");
}

}  // namespace esphome::emontx_ha_bridge
