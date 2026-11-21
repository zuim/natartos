#!/bin/bash
# Build natartos: loader + mods in Docker (no toolchain on NAS)
# Outputs: dist/ with .so files, manifest.json, SHA256SUMS
#
# Sources are mounted read-only; .so output goes directly to dist/
# to avoid Docker root-owned files and Makefile clobbering in mods/.
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASE_DIR="$SCRIPT_DIR/.."
SRC_DIR="$BASE_DIR/src"
MODS_DIR="$BASE_DIR/mods"
DIST_DIR="$BASE_DIR/dist"
DEPLOY_DIR="$BASE_DIR/deploy"

# Clean dist
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"

CC="aarch64-linux-gnu-gcc"
CFLAGS="-shared -fPIC -O2 -fno-builtin -nostdlib"

# Cross-compile in Docker: sources RO, output to /dist
docker run --rm \
  -v "$SRC_DIR:/src:ro" \
  -v "$MODS_DIR:/mods:ro" \
  -v "$BASE_DIR/include:/include:ro" \
  -v "$DIST_DIR:/dist" \
  --platform linux/amd64 gcc:13 bash -c "
  apt-get update -qq && apt-get install -y -qq gcc-aarch64-linux-gnu >/dev/null 2>&1

  echo '=== Building loader ==='
  $CC $CFLAGS -I/include -Wl,-soname,natartos_loader.so -o /dist/natartos_loader.so /src/natartos_loader.c
  echo \"[natartos] natartos_loader.so (\$(stat -c%s /dist/natartos_loader.so) bytes)\"

  echo '=== Building mods ==='
  for dir in /mods/*/; do
    dir=\$(basename \"\$dir\")
    [ ! -f /mods/\$dir/mod.json ] && continue
    # Skip mods with so=null
    if grep -q '\"so\".*null' /mods/\$dir/mod.json; then
      echo \"[skip] \$dir (so=null)\"; continue
    fi
    so=\$(grep '\"so\"' /mods/\$dir/mod.json | head -1 | sed 's/.*\"so\"[^\"]*\"\\([^\"]*\\)\".*/\\1/')
    [ -z \"\$so\" ] && so=\"\${dir}.so\"
    # Source file: \"source\" field in mod.json, or mod.c
    srcfile=\$(grep '\"source\"' /mods/\$dir/mod.json 2>/dev/null | head -1 | sed 's/.*\"source\"[^\"]*\"\\([^\"]*\\)\".*/\\1/')
    [ -z \"\$srcfile\" ] && srcfile=\"mod.c\"
    if [ ! -f /mods/\$dir/\$srcfile ]; then
      echo \"[skip] \$dir (no \$srcfile)\"; continue
    fi
    # Mods with \"libc\":true need libc + pthread
    if grep -q '\"libc\".*true' /mods/\$dir/mod.json 2>/dev/null; then
      MODFLAGS=\"-shared -fPIC -O2 -lpthread -lm\"
    else
      MODFLAGS=\"$CFLAGS\"
    fi
    $CC \$MODFLAGS -I/include -I/mods -Wl,-soname,\$so -o /dist/\$so /mods/\$dir/\$srcfile
    echo \"[natartos] \$so (\$(stat -c%s /dist/\$so) bytes)\"
  done

  chmod 644 /dist/*.so

  # A preloaded .so with an unresolved symbol makes the dynamic linker abort
  # every process it is preloaded into (inject.sh's own date/basename too, so
  # the respawn comes up without mods). -nostdlib can't resolve libgcc helpers
  # such as gcc's outline atomics (__aarch64_swp8_* from __atomic_exchange_n).
  bad=0
  for so in /dist/*.so; do
    u=\$(aarch64-linux-gnu-nm -D --undefined-only \"\$so\" | awk '{print \$2}' | grep -vxE 'dlsym' || true)
    if [ -n \"\$u\" ]; then echo \"[natartos] ERROR: \$so has undefined symbols: \$u\"; bad=1; fi
  done
  [ \$bad = 0 ] || exit 1

  echo '=== Building binaries ==='
  # fb_receiver: static aarch64 binary for PC streaming
  if [ -f /mods/stream/fb_receiver.c ]; then
    $CC -static -O2 -I/include -o /dist/fb_receiver /mods/stream/fb_receiver.c -lm
    chmod 755 /dist/fb_receiver
    echo \"[natartos] fb_receiver (\$(stat -c%s /dist/fb_receiver) bytes)\"
  fi
"

# Build dm_noverity (static aarch64 binary for runtime dm-verity bypass)
docker run --rm \
  -v "$BASE_DIR/tools:/tools:ro" \
  -v "$DIST_DIR:/dist" \
  --platform linux/amd64 gcc:13 bash -c "
  apt-get update -qq && apt-get install -y -qq gcc-aarch64-linux-gnu >/dev/null 2>&1
  aarch64-linux-gnu-gcc -static -O2 -o /dist/dm_noverity /tools/dm_noverity.c
  chmod 644 /dist/dm_noverity
  echo \"[natartos] dm_noverity (\$(stat -c%s /dist/dm_noverity) bytes)\"
"

# Copy deploy scripts to dist
cp "$DEPLOY_DIR/inject.sh" "$DIST_DIR/"

# Generate SHA256SUMS
cd "$DIST_DIR"
sha256sum *.so inject.sh dm_noverity > SHA256SUMS

# --- Copy assets to dist ---
ICONS_DIR="$BASE_DIR/assets/icons"
if [ -d "$ICONS_DIR" ]; then
    mkdir -p "$DIST_DIR/icons"
    cp "$ICONS_DIR"/*.rgba "$DIST_DIR/icons/" 2>/dev/null || true
fi
APPS_DIR="$BASE_DIR/assets/apps"
if [ -d "$APPS_DIR" ]; then
    cp -r "$APPS_DIR" "$DIST_DIR/apps"
fi

# --- eyepiece_bridge (Android NDK, built on the host, not in Docker) ---
# Skipped with a message when no NDK is found ($ANDROID_NDK_HOME or
# ~/Android/Sdk/ndk/*); the mod's menu entry then has nothing to launch.
set +e
bash "$MODS_DIR/eyepiece/bridge/build.sh" "$DIST_DIR/apps/eyepiece"
rc=$?
set -e
if [ "$rc" = 3 ]; then
    echo "[skip] eyepiece_bridge (no Android NDK; set ANDROID_NDK_HOME)"
    rmdir "$DIST_DIR/apps/eyepiece" "$DIST_DIR/apps" 2>/dev/null || true
elif [ "$rc" != 0 ]; then
    echo "[natartos] ERROR: eyepiece_bridge build failed" >&2
    exit "$rc"
fi

# --- Generate manifest.json (schema 1, toolya contract) ---
VERSION="${NATARTOS_VERSION:-dev}"

# Build mods array from mod.json + compiled .so in dist/
MODS_JSON=$(python3 -c "
import json, os, glob
mods_dir = '$MODS_DIR'
dist_dir = '.'
entries = []
for d in sorted(glob.glob(os.path.join(mods_dir, '*/mod.json'))):
    with open(d) as f:
        m = json.load(f)
    so = m.get('so')
    if so is None:
        continue
    if not os.path.isfile(os.path.join(dist_dir, so)):
        continue
    import hashlib
    with open(os.path.join(dist_dir, so), 'rb') as sf:
        sha = hashlib.sha256(sf.read()).hexdigest()
    provides_map = {
        'overlay': 'screen',
        'background': 'background',
        'splash': 'splash',
    }
    entry = {
        'id': m['id'],
        'name': m.get('name', m['id']),
        'description': m.get('notes', ''),
        'provides': provides_map.get(m['id'], None),
        'default': m['id'] in ('overlay', 'popen', 'menu', 'doom', 'stream'),
        'file': so,
        'device_path': '/vendor/natartos/mods/' + so,
        'mode': '0644',
        'sha256': sha,
    }
    entry = {k: v for k, v in entry.items() if v is not None}
    entries.append(entry)
print(json.dumps(entries, indent=4))
")

LOADER_SHA=$(sha256sum natartos_loader.so | cut -d' ' -f1)
INJECT_SHA=$(sha256sum inject.sh | cut -d' ' -f1)

MODS_JSON="$MODS_JSON" \
LOADER_SHA="$LOADER_SHA" INJECT_SHA="$INJECT_SHA" \
VERSION="$VERSION" \
python3 -c '
import json, os
import hashlib
files = [
    {
        "file": "inject.sh",
        "device_path": "/vendor/natartos/inject.sh",
        "mode": "0755",
        "sha256": os.environ["INJECT_SHA"],
    },
]
if os.path.isdir("icons"):
    for f in sorted(os.listdir("icons")):
        if f.endswith(".rgba") and os.path.isfile(os.path.join("icons", f)):
            files.append({
                "file": "icons/" + f,
                "device_path": "/vendor/natartos/icons/" + f,
                "mode": "0644",
                "sha256": hashlib.sha256(open(os.path.join("icons", f), "rb").read()).hexdigest(),
            })
if os.path.isdir("apps"):
    for root, dirs, fnames in os.walk("apps"):
        for f in sorted(fnames):
            rel = os.path.join(root, f)
            mode = "0755" if os.access(rel, os.X_OK) else "0644"
            files.append({
                "file": rel,
                "device_path": "/vendor/natartos/" + rel,
                "mode": mode,
                "sha256": hashlib.sha256(open(rel, "rb").read()).hexdigest(),
            })
manifest = {
    "schema": 1,
    "natartos_version": os.environ["VERSION"],
    "device": "E3T zv902",
    "loader": {
        "file": "natartos_loader.so",
        "device_path": "/vendor/natartos/natartos_loader.so",
        "mode": "0644",
        "sha256": os.environ["LOADER_SHA"],
    },
    "files": files,
    "mods": json.loads(os.environ["MODS_JSON"]),
    "backup": [
        "/vendor/etc/init/dji_glasses_custom.rc",
        "/vendor/etc/selinux/precompiled_sepolicy",
    ],
    "remove": [
        "/vendor/etc/init/persist_adb.rc",
        "/vendor/lib64/natartos_loader.so",
        "/vendor/lib64/hook_debug.so",
        "/vendor/natartos/mods/hook_debug.so",
        "/vendor/lib64/hook_bitmap.so",
    ],
}
with open("manifest.json", "w") as f:
    json.dump(manifest, f, indent=2)
    f.write("\n")
print(json.dumps(manifest, indent=2))
'

cd "$BASE_DIR"
echo ""
echo "=== dist/ ==="
ls -la "$DIST_DIR"
