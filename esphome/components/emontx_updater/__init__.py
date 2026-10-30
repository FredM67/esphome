import esphome.codegen as cg
from esphome.components import emontx, esp32
from esphome.components.api import CONF_CUSTOM_SERVICES, CONF_HOMEASSISTANT_SERVICES
from esphome.components.emontx import CONF_EMONTX_ID
from esphome.components.http_request import CONF_VERIFY_SSL
import esphome.config_validation as cv
from esphome.const import CONF_ID
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@FredM67"]
DEPENDENCIES = ["esp32", "emontx", "http_request", "api"]
DOMAIN = "emontx_updater"

emontx_updater_ns = cg.esphome_ns.namespace("emontx_updater")
EmonTxUpdater = emontx_updater_ns.class_("EmonTxUpdater", cg.Component)

CONF_BOOTLOADER_TIMEOUT = "bootloader_timeout"
CONF_DRY_RUN = "dry_run"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EmonTxUpdater),
        cv.GenerateID(CONF_EMONTX_ID): cv.use_id(emontx.EmonTx),
        cv.Optional(
            CONF_BOOTLOADER_TIMEOUT, default="5s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_DRY_RUN, default=False): cv.boolean,
        cv.Optional(CONF_VERIFY_SSL, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


def final_validate(config: ConfigType) -> ConfigType:
    api_config = fv.full_config.get().get("api", {})
    if not api_config.get(CONF_CUSTOM_SERVICES, False):
        raise cv.Invalid(
            "emontx_updater requires 'custom_services: true' in the 'api:' section "
            "(needed to register the flash_emontx6 service in Home Assistant)."
        )
    if not api_config.get(CONF_HOMEASSISTANT_SERVICES, False):
        raise cv.Invalid(
            "emontx_updater requires 'homeassistant_services: true' in the 'api:' section "
            "(needed to send flash status events to Home Assistant)."
        )
    return config


FINAL_VALIDATE_SCHEMA = final_validate


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_EMONTX_ID])
    var = cg.new_Pvariable(config[CONF_ID], hub)
    await cg.register_component(var, config)
    cg.add(var.set_bootloader_timeout(config[CONF_BOOTLOADER_TIMEOUT]))
    cg.add(var.set_dry_run(config[CONF_DRY_RUN]))
    cg.add(var.set_verify_ssl(config[CONF_VERIFY_SSL]))
    # The download uses esp_http_client directly, so it can't rely on http_request's TLS settings
    esp32.request_http_client()
    if config[CONF_VERIFY_SSL]:
        esp32.require_certificate_bundle()
    else:
        esp32.allow_insecure_tls()
