"""Stable SDK versions allowed by the pinned wheel policy."""
DEFAULT_SDK_VERSION = "1.0.9"
SUPPORTED_SDK_VERSIONS = ("1.0.8", "1.0.9")

def validate_sdk_version(sdk):
    version = getattr(sdk, "__version__", None)
    if version not in SUPPORTED_SDK_VERSIONS:
        raise RuntimeError("unsupported BPX SDK version: {!r}".format(version))
    return version
