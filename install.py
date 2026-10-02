#!/usr/bin/env python3
"""install.py - put stereopsis and the modes in this folder on an EYESY over WiFi, or take them off again.

  python install.py <address>              install, or update an earlier install (shows what it will do, asks first)
  python install.py <address> --check      only show what it would do; changes nothing
  python install.py <address> --uninstall  back to how the EYESY was: Critter & Guitari's engine, their modes as they
                                           were, everything this added removed (shows what it will do, asks first)
  python install.py --find                 look for EYESYs on this computer's network

  --yes         do not ask
  --no-restart  after an install, leave the video engine running as it is (the change takes effect at its next
                start); an uninstall always stops the engine while it works and starts it again
  --force       go on even if the EYESY's software is not the OS v3.1 this was made for

<address> is the EYESY's IP address: on the EYESY press Shift + OSD and choose WiFi, the address is shown there (the
same one its web editor uses, http://<address>/). The EYESY and this computer must be on the same network.
Needs Python 3.7 or newer and nothing else. Everything goes through the EYESY's own web editor; only its SD card's
/sdcard folder is changed (the OS and Critter & Guitari's engine stay as they are).
"""
import concurrent.futures
import hashlib
import json
import os
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
# OS v3.1's stock engine (EYESY_OS 51cc186, engines/python/main.py): how this recognises the software it was made for
V31_ENGINE = "home/music/EYESY_OS/engines/python/main.py"
V31_ENGINE_SHA256 = "101a68e0759194121b85d2b859c6dd4b30bd4d879d922d69a2c29d4583456386"
SD = "sdcard"
ENGINE_DIR = SD + "/stereopsis/engine"
BACKUP_DIR = SD + "/stereopsis/backup/Modes"
MANIFEST = SD + "/stereopsis/INSTALLED.json"
SCALE = SD + "/stereopsis/scale.json"
LAB = "Z - Engine Lab"
BINARY = (".ttf",)
# render sizes for a first install (the page's Size row changes them; they are kept on an update): the heavier modes
# at half size, where they run at 60 frames a second
DEFAULT_SCALE = {"default": "full", "modes": {
    "04 - Swarm - Orb": "640x360", "05 - Swarm - Nebula": "640x360", "06 - Swarm - Cymatics": "640x360",
    "07 - Soundfield Splats 2": "640x360", "10 - Phase Space": "640x360", "T - Woven Feedback": "640x360"}}
PREVIEW_PORT = 8081


def say(*a):
    print(*a, flush=True)


# ------------------------------------------------------------------------------------------- the EYESY's web editor
class Eyesy:
    def __init__(self, address):
        self.address = address                         # an IP address or name, optionally with the editor's :port
        self.host = address.rsplit(":", 1)[0] if address.count(":") == 1 else address
        self.base = "http://%s" % address
        self._dirs = {}

    def http(self, path, data=None, timeout=20, tries=3):
        body = urllib.parse.urlencode(data).encode() if isinstance(data, dict) else data
        last = None
        for i in range(tries):
            try:
                req = urllib.request.Request(self.base + path, data=body)
                with urllib.request.urlopen(req, timeout=timeout) as r:
                    return r.read()
            except urllib.error.HTTPError:
                raise
            except Exception as e:                      # WiFi hiccup: try again
                last = e
                time.sleep(1.0 + i)
        raise last

    def fm(self, **q):
        return self.http("/fmdata?" + urllib.parse.urlencode(q))

    def ls(self, path):
        """{name: "file" | "folder"} for a folder, None if it does not exist (looked up in its parent's listing first,
        so the editor is never asked for a folder that is not there: that leaves an error in the EYESY's console)"""
        if path in self._dirs:
            return self._dirs[path]
        parent, name = path.rsplit("/", 1) if "/" in path else ("#", path)
        if path != "#" and (parent != "#" or name in ("sdcard", "usbdrive")):   # the editor's root lists only these
            above = self.ls(parent)
            if above is None or above.get(name) != "folder":
                self._dirs[path] = None
                return None
        try:
            raw = self.fm(operation="get_node", path=path)
        except urllib.error.HTTPError:
            self._dirs[path] = None
            return None
        nodes = json.loads(raw)
        if isinstance(nodes, str):                     # the editor json.dumps()es, then jsonify()s it again
            nodes = json.loads(nodes)
        self._dirs[path] = {n["name"]: n["type"] for n in nodes}
        return self._dirs[path]

    def exists(self, path):
        parent, name = path.rsplit("/", 1) if "/" in path else ("#", path)
        listing = self.ls(parent)
        return listing is not None and name in listing

    def read(self, path):
        """the file's bytes, or None if there is none"""
        if not self.exists(path):
            return None
        return self.http("/get_file?" + urllib.parse.urlencode({"fpath": path}))

    def mkdir(self, path):
        if self.exists(path):
            return
        parent, name = path.rsplit("/", 1)
        if not self.exists(parent):
            self.mkdir(parent)
        self.fm(operation="create_node", path=parent, name=name)
        self._forget(parent)
        if not self.exists(path):
            raise RuntimeError("could not create %s" % path)

    def save(self, path, data):
        """a text file, written and read back"""
        text = data.decode("utf-8")
        if not text:
            raise RuntimeError("refusing to write an empty file: %s" % path)   # the editor refuses them too
        self.http("/save", {"fpath": path, "content": text})
        self._forget(path.rsplit("/", 1)[0])
        back = self.http("/get_file?" + urllib.parse.urlencode({"fpath": path}))
        if back != data:
            raise RuntimeError("written but read back different: %s" % path)

    def upload(self, path, data):
        """a binary file (the editor's /upload never overwrites: an existing file is deleted first)"""
        folder, name = path.rsplit("/", 1)
        if self.exists(path):
            self.delete(path)
        boundary = uuid.uuid4().hex
        body = (("--%s\r\nContent-Disposition: form-data; name=\"dst\"\r\n\r\n/%s\r\n" % (boundary, folder)).encode()
                + ("--%s\r\nContent-Disposition: form-data; name=\"files[]\"; filename=\"%s\"\r\n"
                   "Content-Type: application/octet-stream\r\n\r\n" % (boundary, name)).encode()
                + data + ("\r\n--%s--\r\n" % boundary).encode())
        req = urllib.request.Request(self.base + "/upload", data=body,
                                     headers={"Content-Type": "multipart/form-data; boundary=" + boundary})
        urllib.request.urlopen(req, timeout=120).read()
        self._forget(folder)
        if self.read(path) != data:
            raise RuntimeError("uploaded but read back different: %s" % path)

    def delete(self, path):
        self.fm(operation="delete_node", path=path)
        self._forget(path.rsplit("/", 1)[0])
        self._forget(path)

    def _forget(self, path):
        for k in [k for k in self._dirs if k == path or k.startswith(path + "/")]:
            del self._dirs[k]

    def stop(self):
        self.http("/stop_video_engine", timeout=60, tries=1)

    def start(self):
        self.http("/start_video_engine", timeout=60, tries=1)

    def state(self):
        """stereopsis's /state.json (or the small page the lab serves under the stock engine), None if nothing answers"""
        try:
            with urllib.request.urlopen("http://%s:%d/state.json" % (self.host, PREVIEW_PORT), timeout=3) as r:
                return json.loads(r.read())
        except Exception:
            return None


# ------------------------------------------------------------------------------------------- what this folder holds
def local_files(folder):
    """relative path -> bytes (text with LF line endings) for every file under folder"""
    out = {}
    root = os.path.join(HERE, folder)
    for dp, dn, fn in os.walk(root):
        dn[:] = [d for d in dn if d not in ("__pycache__", "build") and not d.startswith(".")]
        for f in fn:
            if f.startswith(".") or f.endswith((".pyc", ".so")):
                continue
            p = os.path.join(dp, f)
            data = open(p, "rb").read()
            if not f.endswith(BINARY):
                data = data.replace(b"\r\n", b"\n")
            out[os.path.relpath(p, root).replace(os.sep, "/")] = data
    return out


def sha(data):
    return hashlib.sha256(data).hexdigest()


def load_manifest(ey):
    raw = ey.read(MANIFEST)
    if raw is None:
        return None
    try:
        return json.loads(raw.decode("utf-8"))
    except ValueError:
        return None


def engine_version(data):
    for line in (data or b"").decode("utf-8", "replace").split("\n"):
        if line.startswith("STEREOPSIS_VERSION"):
            return line.split('"')[1]
    return None


# ------------------------------------------------------------------------------------------- looking at the EYESY
def survey(ey, force):
    """what is on the EYESY now; exits with a message when it is not an EYESY this can install on"""
    try:
        top = ey.ls(SD)
    except Exception as e:
        sys.exit("No EYESY web editor answers at http://%s/ (%s).\nCheck the address (Shift + OSD, then WiFi, on the "
                 "EYESY) and that this computer is on the same network." % (ey.address, e))
    if not top or "Modes" not in top or "System" not in top:
        sys.exit("http://%s/ answers, but it does not look like an EYESY on OS v3 (no /sdcard/Modes and System)."
                 % ey.address)
    stock = ey.read(V31_ENGINE)
    os_ok = stock is not None and sha(stock) == V31_ENGINE_SHA256
    if not os_ok and not force:
        sys.exit("This EYESY's software is not the OS v3.1 this was made for (%s).\nUpdate it to OS v3.1 first (see "
                 "the README), or run again with --force to try anyway: the stock engine is never changed, so the "
                 "EYESY can always fall back to it." % ("no stock engine found where v3.1 has it" if stock is None
                                                         else "its engine differs from v3.1's"))
    usb = ey.ls("usbdrive")
    modes = {n for n, t in (ey.ls(SD + "/Modes") or {}).items() if t == "folder"}
    installed = engine_version(ey.read(ENGINE_DIR + "/main.py"))
    return {"os_ok": os_ok, "usb_modes": bool(usb and "Modes" in usb), "modes": modes, "installed": installed,
            "manifest": load_manifest(ey)}


def plan_install(ey, info):
    """the list of steps (kind, device path, data, note) and the summary lines"""
    steps, notes = [], []
    manifest = info["manifest"] or {}
    # 1. the engine
    eng = local_files("engine")
    new_eng = changed_eng = 0
    for rel, data in sorted(eng.items()):
        path = ENGINE_DIR + "/" + rel
        old = ey.read(path)
        if old == data:
            continue
        steps.append(("upload" if rel.endswith(BINARY) else "save", path, data, None))
        new_eng += old is None
        changed_eng += old is not None
    version = engine_version(eng.get("main.py"))
    notes.append("engine      stereopsis %s: %d files (%d new, %d changed, %d already there) -> /%s"
                 % (version, len(eng), new_eng, changed_eng, len(eng) - new_eng - changed_eng, ENGINE_DIR))
    # 2. our modes and the lab
    ours = {}
    for rel, data in local_files("modes").items():
        mode, f = rel.split("/", 1)
        ours.setdefault(mode, {})[f] = data
    created = list(manifest.get("created_folders", []))
    n_new = n_changed = 0
    for mode in sorted(ours):
        folder = SD + "/Modes/" + mode
        if mode not in info["modes"]:
            steps.append(("mkdir", folder, None, None))
            if mode not in created:
                created.append(mode)
            n_new += 1
        else:
            for f, data in ours[mode].items():
                if ey.read(folder + "/" + f) != data:
                    n_changed += 1
                    break
        for f, data in sorted(ours[mode].items()):
            if mode not in info["modes"] or ey.read(folder + "/" + f) != data:
                steps.append(("save", folder + "/" + f, data, None))
    notes.append("our modes   %d folders (%d new, %d changed): %s .. %s, and %s (it starts stereopsis)"
                 % (len(ours), n_new, n_changed, sorted(ours)[0], sorted(m for m in ours if m != LAB)[-1], LAB))
    switch = SD + "/Modes/%s/STEREOPSIS" % LAB
    if (ey.read(switch) or b"").strip() != b"boot":
        steps.append(("save", switch, b"boot\n", None))
    notes.append("boot        %s/STEREOPSIS = boot: stereopsis starts at every boot (hold Shift + Persist for a "
                 "second to switch to the stock engine and back)" % LAB)
    # 3. Critter & Guitari's factory modes: the 60 fps versions, only over the exact published files
    pub = json.loads(open(os.path.join(HERE, "factory-modes", "published.json"), encoding="utf-8").read())["modes"]
    fac_done = dict(manifest.get("factory", {}))
    adapted, added, kept_as_is, already = [], [], [], 0
    for mode, meta in sorted(pub.items()):
        ours_main = open(os.path.join(HERE, "factory-modes", mode, "main.py"), "rb").read().replace(b"\r\n", b"\n")
        path = SD + "/Modes/%s/main.py" % mode
        if mode not in info["modes"]:
            if meta["files_published"] == 1:           # the newer modes are one file each: add them
                steps.append(("mkdir", SD + "/Modes/" + mode, None, None))
                steps.append(("save", path, ours_main, None))
                fac_done[mode] = {"added": True, "installed_sha256": sha(ours_main)}
                added.append(mode)
            continue
        cur = ey.read(path)
        if cur is None:
            continue
        if cur == ours_main:
            already += meta["adapted"]                 # (an unadapted mode as published needs nothing)
            continue
        mine_before = fac_done.get(mode, {}).get("installed_sha256")
        if sha(cur) == meta["published_sha256"]:
            if not meta["adapted"]:
                continue
            backup = BACKUP_DIR + "/%s/main.py" % mode
            if ey.read(backup) is None:
                steps.append(("mkdir", BACKUP_DIR + "/" + mode, None, None))
                steps.append(("save", backup, cur, None))
            steps.append(("save", path, ours_main, None))
            fac_done[mode] = dict(fac_done.get(mode, {}), backup=True, installed_sha256=sha(ours_main))
            adapted.append(mode)
        elif mine_before is not None and sha(cur) == mine_before:          # an earlier install's version: update
            steps.append(("save", path, ours_main, None))
            fac_done[mode] = dict(fac_done[mode], installed_sha256=sha(ours_main))
            adapted.append(mode)
        elif meta["adapted"]:
            kept_as_is.append(mode)
    notes.append("factory     %d modes get their 60 fps version (originals kept in /%s), %d newer modes added%s"
                 % (len(adapted), BACKUP_DIR, len(added), ", %d already done" % already if already else ""))
    if kept_as_is:
        notes.append("            left as they are (not the published version, so not replaced; they may move twice "
                     "as fast under stereopsis): %s" % ", ".join(kept_as_is))
    # 4. render sizes: only on a first install
    if ey.read(SCALE) is None:
        steps.append(("save", SCALE, (json.dumps(DEFAULT_SCALE, indent=2) + "\n").encode(), None))
        notes.append("sizes       scale.json: %s at 640x360, the rest at the full size (the page's Size row "
                     "changes them)" % ", ".join(sorted(DEFAULT_SCALE["modes"])))
    else:
        notes.append("sizes       scale.json kept as it is")
    new_manifest = {"stereopsis": version, "installed": time.strftime("%Y-%m-%d %H:%M"),
                    "first_installed": manifest.get("first_installed", time.strftime("%Y-%m-%d %H:%M")),
                    "created_folders": created, "factory": fac_done}
    return steps, notes, new_manifest


def plan_uninstall(ey, info, force):
    manifest = info["manifest"]
    if manifest is None and not force:
        sys.exit("No record of an install by this script on this EYESY (/%s is missing), so it cannot tell what to put "
                 "back.\nRun again with --force to remove stereopsis and our modes anyway (Critter & Guitari's modes "
                 "would stay as they are now)." % MANIFEST)
    manifest = manifest or {}
    ours = set(os.listdir(os.path.join(HERE, "modes")))
    steps, notes = [], []
    restored, kept = [], []
    backups = ey.ls(BACKUP_DIR) or {}
    for mode in sorted(backups):
        original = ey.read(BACKUP_DIR + "/%s/main.py" % mode)
        cur = ey.read(SD + "/Modes/%s/main.py" % mode)
        if original is None or cur is None:
            continue
        mine = manifest.get("factory", {}).get(mode, {}).get("installed_sha256")
        if cur == original:
            continue
        if mine is None or sha(cur) == mine:
            steps.append(("save", SD + "/Modes/%s/main.py" % mode, original, None))
            restored.append(mode)
        else:                                          # edited since: keep the edit, put the original beside it
            steps.append(("save", SD + "/Modes/%s/main.py.original" % mode, original, None))
            kept.append(mode)
    removed_factory = []
    for mode, meta in sorted(manifest.get("factory", {}).items()):
        if meta.get("added"):
            cur = ey.read(SD + "/Modes/%s/main.py" % mode)
            if cur is not None and sha(cur) == meta.get("installed_sha256"):
                steps.append(("delete", SD + "/Modes/" + mode, None, None))
                removed_factory.append(mode)
    created = manifest.get("created_folders") or sorted(ours)
    gone = [m for m in created if m in ours and m in info["modes"]]
    for mode in gone:
        steps.append(("delete", SD + "/Modes/" + mode, None, None))
    if ey.exists(SD + "/stereopsis"):
        steps.append(("delete", SD + "/stereopsis", None, None))
    notes.append("factory     %d modes back to their original main.py%s, %d added ones removed"
                 % (len(restored), (" (%d edited since: the original saved beside them as main.py.original)" % len(kept)
                                    if kept else ""), len(removed_factory)))
    notes.append("our modes   %d folders removed: %s" % (len(gone), ", ".join(gone) or "none"))
    notes.append("engine      /sdcard/stereopsis removed (the engine, its settings and the backups)")
    if not manifest:
        notes.append("            (no install record: Critter & Guitari's modes stay as they are now)")
    return steps, notes


def run(ey, steps):
    n = len(steps)
    sys.stdout.write("  working ")
    sys.stdout.flush()
    for i, (kind, path, data, _) in enumerate(steps, 1):
        if kind == "mkdir":
            ey.mkdir(path)
        elif kind == "save":
            ey.mkdir(path.rsplit("/", 1)[0])           # (a no-op when the folder is there)
            ey.save(path, data)
        elif kind == "upload":
            ey.mkdir(path.rsplit("/", 1)[0])
            ey.upload(path, data)
        elif kind == "delete":
            ey.delete(path)
        if i < n and i * 10 // n != (i - 1) * 10 // n:          # every tenth of the way
            sys.stdout.write("%d%% " % (i * 10 // n * 10))
            sys.stdout.flush()
    say("done (%d steps)" % n)


def restart_and_wait(ey, expect_stereopsis):
    say("Restarting the video engine...")
    ey.stop()
    time.sleep(2)
    ey.start()
    if not expect_stereopsis:
        say("Started. The EYESY runs Critter & Guitari's engine again (its modes appear in ~30 s).")
        return True
    say("Waiting for stereopsis (the first start compiles its display and audio code on the EYESY: 1-2 minutes)")
    t0 = time.time()
    stock_since = None
    while time.time() - t0 < 360:
        s = ey.state()
        if s and s.get("engine") == "stereopsis":
            say("\nstereopsis is running: %s, %s modes, on %s." % (s.get("version"), s.get("mode_count"), s.get("mode")))
            say("The page (live picture + every control): http://%s:%d/" % (ey.host, PREVIEW_PORT))
            say("The modes' renderers now compile on the EYESY in the background, one at a time (~3 minutes in all);\n"
                "each mode shows a simple preview until its own is ready.")
            return True
        if s and s.get("engine") == "stock":
            stock_since = stock_since or time.time()
            if time.time() - stock_since > 60:
                break
        sys.stdout.write(".")
        sys.stdout.flush()
        time.sleep(5)
    say("\nstereopsis did not come up. The EYESY is still usable: Critter & Guitari's engine runs when stereopsis "
        "cannot\n(it switches itself off if it fails to start). Open http://%s/ (the EYESY's web editor) and look at "
        "the console\nfor errors, then run this again with --check." % ey.address)
    return False


# ------------------------------------------------------------------------------------------- finding EYESYs
def probe(ip):
    """ip if an EYESY's web editor answers there (its /sdcard holds Modes and System), else None"""
    try:
        with urllib.request.urlopen("http://%s/fmdata?operation=get_node&path=sdcard" % ip, timeout=1.5) as r:
            nodes = json.loads(r.read())
            if isinstance(nodes, str):
                nodes = json.loads(nodes)
            names = {n.get("name") for n in nodes}
            return ip if {"Modes", "System"} <= names else None
    except Exception:
        return None


def find():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.0.2.1", 9))                    # no packet is sent: this only picks the outgoing interface
        mine = s.getsockname()[0]
        s.close()
    except OSError:
        sys.exit("Could not tell this computer's network address.")
    prefix = mine.rsplit(".", 1)[0]
    say("Looking for EYESYs on %s.1-254 (a few seconds)..." % prefix)
    with concurrent.futures.ThreadPoolExecutor(64) as pool:
        found = [ip for ip in pool.map(probe, ["%s.%d" % (prefix, i) for i in range(1, 255)]) if ip]
    if not found:
        say("None found. Is the EYESY on WiFi (Shift + OSD, then WiFi) and this computer on the same network?")
    for ip in found:
        say("EYESY at %s  ->  python install.py %s" % (ip, ip))


# ------------------------------------------------------------------------------------------- main
def main(argv):
    flags = {a for a in argv if a.startswith("--")}
    args = [a for a in argv if not a.startswith("--")]
    known = {"--check", "--uninstall", "--find", "--yes", "--no-restart", "--force"}
    if flags - known or ("--find" not in flags and len(args) != 1):
        sys.exit(__doc__)
    if "--find" in flags:
        return find()
    if sys.version_info < (3, 7):
        sys.exit("Python 3.7 or newer is needed.")
    ey = Eyesy(args[0])
    say("Looking at the EYESY at %s ..." % ey.address)
    info = survey(ey, "--force" in flags)
    say("EYESY at %s: %s, %d modes, stereopsis %s%s." % (
        ey.address, "OS v3.1" if info["os_ok"] else "NOT the OS v3.1 this was made for (--force)", len(info["modes"]),
        info["installed"] or "not installed", " (installed by this script)" if info["manifest"] else ""))
    if info["usb_modes"]:
        say("NOTE: a USB drive with a Modes folder is plugged in, and the EYESY runs from it instead of its SD card. "
            "This installs on the SD card: take the USB drive out (and restart) to use it.")
    uninstall = "--uninstall" in flags
    if uninstall:
        steps, notes = plan_uninstall(ey, info, "--force" in flags)
        manifest = None
    else:
        steps, notes, manifest = plan_install(ey, info)
    say("\n" + ("To uninstall:" if uninstall else "To install:"))
    for line in notes:
        say("  " + line)
    restart = "--no-restart" not in flags
    if not steps:
        say("\nNothing to change: the EYESY already has all of it.")
        return 0
    say("  then        %s" % ("restart the video engine" if restart or uninstall else
                              "nothing more (--no-restart: it takes effect at the engine's next start)"))
    say("  (%d steps)" % len(steps))
    if "--check" in flags:
        say("\n--check: nothing was changed.")
        return 0
    if "--yes" not in flags:
        try:
            ok = input("\nGo ahead? [y/N] ").strip().lower() in ("y", "yes")
        except EOFError:
            ok = False
        if not ok:
            say("Nothing was changed.")
            return 1
    if uninstall:
        say("Stopping the video engine while files are restored and removed...")
        ey.stop()
        time.sleep(2)
        try:
            run(ey, steps)
        finally:
            time.sleep(1)
            ey.start()
            say("Started again: Critter & Guitari's engine, as before the install.")
        return 0
    # an install: the record first (so an interrupted install can still be undone), then the files, the record again
    ey.mkdir(SD + "/stereopsis")
    record = dict(manifest, complete=False)
    ey.save(MANIFEST, (json.dumps(record, indent=1, sort_keys=True) + "\n").encode())
    run(ey, steps)
    ey.save(MANIFEST, (json.dumps(dict(manifest, complete=True), indent=1, sort_keys=True) + "\n").encode())
    say("Files done.")
    if restart:
        return 0 if restart_and_wait(ey, True) else 1
    say("It takes effect at the video engine's next start (Shift + OSD > System > Restart Video, or power-cycle).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        sys.exit("\nStopped. Run the same command again to finish (it picks up where it left off).")
