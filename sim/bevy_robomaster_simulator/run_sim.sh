#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"
[ -f "$HOME/.cargo/env" ] && source "$HOME/.cargo/env"
RUST_SYSROOT="$(rustc --print sysroot)"
export DISPLAY="${DISPLAY:-:0}"
export BEVY_ASSET_ROOT="$DIR"
export LD_LIBRARY_PATH="$DIR/target/release:$DIR/target/release/deps:$RUST_SYSROOT/lib:$RUST_SYSROOT/lib/rustlib/x86_64-unknown-linux-gnu/lib:${LD_LIBRARY_PATH:-}"
exec "$DIR/target/release/daedalus" "$@"
