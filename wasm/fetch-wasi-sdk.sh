#!/usr/bin/env bash
# Fetch the pinned wasi-sdk release used by `make wasm` and unpack it into
# build/wasi-sdk/ (gitignored). Picks the tarball for the running OS/arch,
# verifies its sha256 against the hashes pinned below, and skips the download
# if a matching SDK is already unpacked there.
#
#   wasm/fetch-wasi-sdk.sh
#
# Override the destination with WASI_SDK_DEST; override the version by
# editing WASI_SDK_VERSION and adding a hash line, taken from the per-asset
# digests GitHub publishes for the release:
#   gh api repos/WebAssembly/wasi-sdk/releases/tags/wasi-sdk-<N> \
#     --jq '.assets[] | "\(.name) \(.digest)"'
set -euo pipefail

WASI_SDK_VERSION=34.0
WASI_SDK_DEST=${WASI_SDK_DEST:-build/wasi-sdk}
RELEASE_URL=https://github.com/WebAssembly/wasi-sdk/releases/download/wasi-sdk-34

# name -> sha256, one line per platform/arch the release ships
sha256_for() {
    case "$1" in
        x86_64-linux) echo "b761e3a0721dbae9c09a0059e5fdb2bf917d1b4a8a7b430fb3b5aafb0984b2c4" ;;
        arm64-linux) echo "f7e243dff54d60bcc576e94d6166b69f410f2500ae4a9ceef34315be10e77971" ;;
        arm64-macos) echo "9c59398106b417f8f14913380fdf0097a8cc0ff4af9eb3ce0065a859e88d49e9" ;;
        x86_64-macos) echo "87d27fa8adc68dee59bfbf2e22a6d34ef717c34d6bf1d8af2a56fc929d9ce0eb" ;;
        *) return 1 ;;
    esac
}

os=$(uname -s)
arch=$(uname -m)
case "$os" in
    Linux) platform=linux ;;
    Darwin) platform=macos ;;
    *) echo "fetch-wasi-sdk.sh: unsupported OS '$os'" >&2; exit 1 ;;
esac
case "$arch" in
    x86_64|amd64) sdk_arch=x86_64 ;;
    aarch64|arm64) sdk_arch=arm64 ;;
    *) echo "fetch-wasi-sdk.sh: unsupported arch '$arch'" >&2; exit 1 ;;
esac

variant="${sdk_arch}-${platform}"
sha256=$(sha256_for "$variant") || {
    echo "fetch-wasi-sdk.sh: no pinned wasi-sdk-$WASI_SDK_VERSION release for $variant" >&2
    exit 1
}
archive="wasi-sdk-${WASI_SDK_VERSION}-${variant}.tar.gz"

if [ -f "$WASI_SDK_DEST/VERSION" ] && grep -q "^${WASI_SDK_VERSION}" "$WASI_SDK_DEST/VERSION" 2>/dev/null; then
    echo "fetch-wasi-sdk.sh: $WASI_SDK_DEST already has wasi-sdk $WASI_SDK_VERSION, skipping"
    exit 0
fi

tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT

echo "fetch-wasi-sdk.sh: downloading $archive"
curl -fSL -o "$tmpdir/$archive" "$RELEASE_URL/$archive"

if command -v sha256sum >/dev/null 2>&1; then
    echo "$sha256  $tmpdir/$archive" | sha256sum -c -
else
    echo "$sha256  $tmpdir/$archive" | shasum -a 256 -c -
fi

rm -rf "$WASI_SDK_DEST"
mkdir -p "$WASI_SDK_DEST"
tar xzf "$tmpdir/$archive" -C "$WASI_SDK_DEST" --strip-components=1

echo "fetch-wasi-sdk.sh: unpacked wasi-sdk $WASI_SDK_VERSION ($variant) into $WASI_SDK_DEST"
