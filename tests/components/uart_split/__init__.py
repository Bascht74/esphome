# This file's presence makes pytest treat this directory as a package named
# "uart_split"; required for cpp unit testing.
import esphome.codegen as cg
from esphome.types import ConfigType
from tests.testing_helpers import ComponentManifestOverride


def override_manifest(manifest: ComponentManifestOverride) -> None:
    manifest.multi_conf = False

    async def to_code_testing(config: ConfigType) -> None:
        # The gtests build the plain copy and the arbiter.
        cg.add_define("USE_UART_SPLIT_ARBITER")

    manifest.to_code = to_code_testing
