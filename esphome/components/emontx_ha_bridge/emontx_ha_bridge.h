#pragma once

#include "esphome/core/component.h"
#include "esphome/components/emontx/emontx.h"
#include "esphome/components/api/custom_api_device.h"

namespace esphome::emontx_ha_bridge {

/**
 * @class EmonTxHaBridge
 * @brief Bridges the EmonTx component to Home Assistant via the native API.
 *
 * Fires esphome.emontx_raw for every raw serial line received and
 * esphome.emontx_json for every successfully parsed JSON frame.
 * Also auto-registers a send_command service so Home Assistant can
 * send configuration commands back to the emonTx hardware.
 */
class EmonTxHaBridge : public Component, public api::CustomAPIDevice {
 public:
  void set_emontx(emontx::EmonTx *emontx) { this->emontx_ = emontx; }

  void setup() override;
  void dump_config() override;

 protected:
  emontx::EmonTx *emontx_{nullptr};

  /// Callback registered as the HA send_command service handler.
  void on_send_command_(std::string command) { this->emontx_->send_command(command); }  // NOLINT
};

}  // namespace esphome::emontx_ha_bridge
