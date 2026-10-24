from tests.testing_helpers import ComponentManifestOverride, add_defines_to_code


def override_manifest(manifest: ComponentManifestOverride) -> None:
    # Only a platform with speeds emits USE_COVER_SPEED; the unit-test build has none.
    add_defines_to_code(manifest, "USE_COVER_SPEED")
