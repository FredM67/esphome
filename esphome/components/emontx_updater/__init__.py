import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import emontx
from esphome.const import CONF_ID
from esphome.types import ConfigType

DEPENDENCIES = ["esp32", "emontx", "http_request", "api"]
CODEOWNERS = ["@FredM67"]

emontx_updater_ns = cg.esphome_ns.namespace("emontx_updater")
EmonTxUpdater = emontx_updater_ns.class_("EmonTxUpdater", cg.Component)

http_request_ns = cg.esphome_ns.namespace("http_request")
HttpRequestComponent = http_request_ns.class_("HttpRequestComponent")

CONF_EMONTX_ID = "emontx_id"
CONF_HTTP_REQUEST_ID = "http_request_id"
CONF_BOOTLOADER_TIMEOUT = "bootloader_timeout"
CONF_DRY_RUN = "dry_run"
CONF_VERIFY_SSL = "verify_ssl"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EmonTxUpdater),
            cv.GenerateID(CONF_EMONTX_ID): cv.use_id(emontx.EmonTx),
            cv.GenerateID(CONF_HTTP_REQUEST_ID): cv.use_id(HttpRequestComponent),
            cv.Optional(CONF_BOOTLOADER_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DRY_RUN, default=False): cv.boolean,
            cv.Optional(CONF_VERIFY_SSL, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


def final_validate(config: ConfigType) -> ConfigType:
    full_config = fv.full_config.get()

    api_config = full_config.get("api", {})
    if not api_config.get("custom_services", False):
        raise cv.Invalid(
            "emontx_updater requires 'custom_services: true' in the 'api:' section "
            "(needed to register the flash_emontx6 service in Home Assistant)."
        )

    return config


FINAL_VALIDATE_SCHEMA = final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_EMONTX_ID])
    cg.add(var.set_emontx(hub))

    http_request = await cg.get_variable(config[CONF_HTTP_REQUEST_ID])
    cg.add(var.set_http_request(http_request))

    cg.add(var.set_bootloader_timeout(config[CONF_BOOTLOADER_TIMEOUT]))
    cg.add(var.set_dry_run(config[CONF_DRY_RUN]))
    cg.add(var.set_verify_ssl(config[CONF_VERIFY_SSL]))
