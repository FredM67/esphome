import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import emontx
from esphome.const import CONF_ID, CONF_TX_PIN, CONF_UART_ID
import esphome.final_validate as fv
from esphome.types import ConfigType

DEPENDENCIES = ["emontx", "api"]
CODEOWNERS = ["@FredM67"]

emontx_ha_bridge_ns = cg.esphome_ns.namespace("emontx_ha_bridge")
EmonTxHaBridge = emontx_ha_bridge_ns.class_("EmonTxHaBridge", cg.Component)

CONF_EMONTX_ID = "emontx_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EmonTxHaBridge),
            cv.Required(CONF_EMONTX_ID): cv.use_id(emontx.EmonTx),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


def final_validate(config: ConfigType) -> ConfigType:
    full_config = fv.full_config.get()

    # Validate API options required by this component
    api_config = full_config.get("api", {})
    if not api_config.get("homeassistant_services", False):
        raise cv.Invalid(
            "emontx_ha_bridge requires 'homeassistant_services: true' in the 'api:' section "
            "(needed to fire esphome.emontx_raw and esphome.emontx_json events)."
        )
    if not api_config.get("custom_services", False):
        raise cv.Invalid(
            "emontx_ha_bridge requires 'custom_services: true' in the 'api:' section "
            "(needed to auto-register the send_command service in Home Assistant)."
        )

    # Validate that the emontx hub's UART has a TX pin configured.
    # The send_command service writes to UART TX, so without it the service silently does nothing.
    emontx_id = str(config[CONF_EMONTX_ID])
    for emontx_conf in full_config.get("emontx", []):
        if str(emontx_conf.get(CONF_ID)) == emontx_id:
            uart_id = emontx_conf.get(CONF_UART_ID)
            for uart_conf in full_config.get("uart", []):
                if uart_conf.get(CONF_ID) == uart_id:
                    if CONF_TX_PIN not in uart_conf:
                        raise cv.Invalid(
                            f"emontx_ha_bridge requires UART '{uart_id}' to have a 'tx_pin' "
                            "configured (needed to send commands via the send_command service "
                            "in Home Assistant).",
                            path=[CONF_EMONTX_ID],
                        )
                    break
            break

    return config


FINAL_VALIDATE_SCHEMA = final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_EMONTX_ID])
    cg.add(var.set_emontx(hub))
