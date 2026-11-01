#!/usr/bin/env bash
# Bootstrap yosys + nextpnr-ecp5 + ecppack + verilator + iverilog from OSS CAD Suite
# (portable, matched versions).
#
# gateware/tools prepends gateware/tools/bin to PATH so synthesis and verification use these
# wrappers instead of stale /usr/local or mismatched apt tools.
#
# Usage:
#   gateware/tools/ensure_fpga_toolchain.sh
#   source gateware/tools/ensure_fpga_toolchain.sh && ensure_fpga_toolchain
#
# Existing OSS CAD tree elsewhere:
#   OSS_CAD_ROOT=/opt/oss-cad-suite gateware/tools/ensure_fpga_toolchain.sh
#
# Opt out (use system PATH tools — not recommended):
#   FPGA_TOOLCHAIN=system gateware/tools/ensure_fpga_toolchain.sh
set -euo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$TOOLS/bin"

FPGA_TOOLCHAIN="${FPGA_TOOLCHAIN:-oss-cad}"
OSS_CAD_ROOT="${OSS_CAD_ROOT:-${NEXTPNR_OSS_CAD_ROOT:-$TOOLS/oss-cad-suite}}"
OSS_CAD_RELEASE="${OSS_CAD_RELEASE:-${NEXTPNR_OSS_CAD_RELEASE:-latest}}"
DOWNLOAD_OSS_CAD="${DOWNLOAD_OSS_CAD:-${NEXTPNR_DOWNLOAD_OSS_CAD:-1}}"

# ECP5 (ULX3S) synthesis + Verilator/iverilog simulation. Xilinx 7-series
# (PCIe Squirrel) uses Vivado, installed separately.
FPGA_TOOLS=(yosys nextpnr-ecp5 ecppack verilator iverilog)

_fpga_die() { echo "error: $*" >&2; exit 1; }

_fpga_need() {
    command -v "$1" >/dev/null 2>&1 || _fpga_die "missing command: $1"
}

oss_cad_arch() {
    case "$(uname -m)" in
        x86_64|amd64) echo "linux-x64" ;;
        aarch64|arm64) echo "linux-arm64" ;;
        *) _fpga_die "unsupported CPU for OSS CAD Suite: $(uname -m)" ;;
    esac
}

oss_cad_release_slug() {
    printf '%s' "${1//-/}"
}

oss_cad_resolve_release() {
    local arch="$1" tag="${OSS_CAD_RELEASE}" slug asset

    if [[ "$tag" == "latest" ]]; then
        _fpga_need python3
        read -r tag asset < <(curl -fsSL \
            "https://api.github.com/repos/YosysHQ/oss-cad-suite-build/releases?per_page=30" \
            | python3 -c "
import json, sys
arch = sys.argv[1]
for release in json.load(sys.stdin):
    tag = release['tag_name']
    slug = tag.replace('-', '')
    expected = f'oss-cad-suite-{arch}-{slug}.tgz'
    for asset in release.get('assets', []):
        if asset['name'] == expected:
            print(tag, asset['name'])
            sys.exit(0)
    for asset in release.get('assets', []):
        name = asset['name']
        if arch in name and name.endswith('.tgz'):
            print(tag, name)
            sys.exit(0)
sys.exit(1)
" "$arch") || _fpga_die "could not find OSS CAD Suite release for ${arch}"
        printf '%s %s\n' "$tag" "$asset"
        return 0
    fi

    slug="$(oss_cad_release_slug "$tag")"
    asset="oss-cad-suite-${arch}-${slug}.tgz"
    printf '%s %s\n' "$tag" "$asset"
}

download_oss_cad_suite() {
    local arch tag asset url tar tool
    arch="$(oss_cad_arch)"
    read -r tag asset < <(oss_cad_resolve_release "$arch")
    url="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/${tag}/${asset}"
    tar="$TOOLS/${asset}.download"

    _fpga_need curl
    _fpga_need tar
    mkdir -p "$TOOLS"

    echo "==> downloading OSS CAD Suite ${tag} (~700 MiB, one-time)"
    echo "    ${url}"
    if [[ -t 2 && "${CURL_QUIET:-0}" != 1 ]]; then
        curl -fL --progress-bar -o "$tar" "$url" || {
            rm -f "$tar"
            _fpga_die "download failed: ${url}"
        }
        echo
    else
        curl -fsSL -o "$tar" "$url" || {
            rm -f "$tar"
            _fpga_die "download failed: ${url}"
        }
    fi
    [[ -s "$tar" ]] || _fpga_die "download empty: ${tar}"

    rm -rf "$OSS_CAD_ROOT"
    echo "==> extracting OSS CAD Suite into $OSS_CAD_ROOT"
    tar xzf "$tar" -C "$TOOLS" || {
        rm -f "$tar"
        _fpga_die "extract failed: ${tar}"
    }
    rm -f "$tar"

    for tool in "${FPGA_TOOLS[@]}"; do
        [[ -x "$OSS_CAD_ROOT/bin/$tool" ]] \
            || _fpga_die "OSS CAD extract missing $OSS_CAD_ROOT/bin/$tool"
    done
    echo "==> OSS CAD Suite ready: $OSS_CAD_ROOT"
}

oss_cad_smoke_test() {
    local root="$1"
    local tool ver

    for tool in "${FPGA_TOOLS[@]}"; do
        [[ -x "$root/bin/$tool" ]] || return 1
    done

    ver="$("$root/bin/yosys" -V 2>&1)" || return 1
    [[ "$ver" == *"Yosys"* ]] || return 1

    ver="$("$root/bin/nextpnr-ecp5" --version 2>&1)" || return 1
    [[ "$ver" == *"nextpnr"* ]] || [[ "$ver" == *"Next Generation"* ]] || return 1

    ver="$("$root/bin/ecppack" 2>&1)" || true
    [[ "$ver" == *"ecppack"* ]] || [[ "$ver" == *"Trellis"* ]] || return 1

    return 0
}

install_fpga_wrappers() {
    local root="$1" tool wrapper
    mkdir -p "$BIN"
    for tool in "${FPGA_TOOLS[@]}"; do
        wrapper="$BIN/$tool"
        cat >"$wrapper" <<EOF
#!/usr/bin/env bash
# gateware FPGA toolchain wrapper → OSS CAD Suite
exec "$root/bin/$tool" "\$@"
EOF
        chmod 755 "$wrapper"
    done
    echo "==> FPGA toolchain wrappers in $BIN"
    echo "    yosys              : $("$BIN/yosys" -V 2>&1 | head -1)"
    echo "    nextpnr-ecp5       : $("$BIN/nextpnr-ecp5" --version 2>&1 | head -1)"
    echo "    ecppack            : $("$BIN/ecppack" 2>&1 | grep -i ecppack | head -1)"
    echo "    verilator          : $("$BIN/verilator" --version 2>&1 | head -1)"
    echo "    iverilog           : $("$BIN/iverilog" -V 2>&1 | head -1)"
}

ensure_fpga_toolchain() {
    if [[ "$FPGA_TOOLCHAIN" == "system" ]]; then
        echo "==> FPGA_TOOLCHAIN=system — using yosys/nextpnr/ecppack from PATH"
        for tool in "${FPGA_TOOLS[@]}"; do
            command -v "$tool" >/dev/null 2>&1 \
                || _fpga_die "missing $tool on PATH (or run without FPGA_TOOLCHAIN=system)"
        done
        return 0
    fi

    if oss_cad_smoke_test "$OSS_CAD_ROOT"; then
        install_fpga_wrappers "$OSS_CAD_ROOT"
        return 0
    fi

    if [[ "${DOWNLOAD_OSS_CAD}" != 1 ]]; then
        _fpga_die "OSS CAD Suite not ready at $OSS_CAD_ROOT (set DOWNLOAD_OSS_CAD=1)"
    fi

    download_oss_cad_suite
    oss_cad_smoke_test "$OSS_CAD_ROOT" \
        || _fpga_die "OSS CAD Suite smoke test failed under $OSS_CAD_ROOT"
    install_fpga_wrappers "$OSS_CAD_ROOT"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    ensure_fpga_toolchain
fi
