"""Sandboxed file store behind /files (CAD meshes pushed by other machines).

Every client path goes through FileStore.resolve(): components must match
[A-Za-z0-9][A-Za-z0-9._-]* (no "..", no hidden files) and the real path must
stay inside the root, so symlinks cannot lead out of it. Uploads are written
to a temporary file in the target directory and renamed into place, so a
reader (a /scene batch parsing a mesh) never sees half a file. sha256 digests
are cached per (size, mtime) to keep listings of big CAD files cheap.
"""
import fnmatch
import hashlib
import os
import re
import shutil
import tempfile
import time

_PART = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class FileStoreError(Exception):
    """A client error; ``status`` is the HTTP status to answer with."""

    def __init__(self, status, code, msg):
        super().__init__(msg)
        self.status, self.code, self.msg = status, code, msg


class FileStore:
    def __init__(self, root, max_file_bytes=100 * 2 ** 20, quota_bytes=2 * 2 ** 30):
        self.root = os.path.realpath(root)
        os.makedirs(self.root, exist_ok=True)
        self.max_file_bytes = int(max_file_bytes)
        self.quota_bytes = int(quota_bytes)
        self._sha = {}            # abs path -> (size, mtime_ns, hex digest)

    # ---- paths ----

    def resolve(self, rel):
        """Absolute path of a client path ('' or '/' = the root); 400 when it
        has a bad component or leaves the root."""
        rel = (rel or "").strip().strip("/")
        if not rel:
            return self.root
        parts = rel.split("/")
        if any(not _PART.match(p) for p in parts):
            raise FileStoreError(400, "bad_path", "path components must match [A-Za-z0-9][A-Za-z0-9._-]* "
                                                  "(no '..', no hidden files)")
        path = os.path.realpath(os.path.join(self.root, *parts))
        if path != self.root and not path.startswith(self.root + os.sep):
            raise FileStoreError(400, "bad_path", "path leaves the file store")
        return path

    def relpath(self, abs_path):
        return os.path.relpath(abs_path, self.root).replace(os.sep, "/")

    # ---- metadata ----

    def sha256(self, abs_path):
        st = os.stat(abs_path)
        hit = self._sha.get(abs_path)
        if hit and hit[0] == st.st_size and hit[1] == st.st_mtime_ns:
            return hit[2]
        h = hashlib.sha256()
        with open(abs_path, "rb") as f:
            for chunk in iter(lambda: f.read(2 ** 20), b""):
                h.update(chunk)
        self._sha[abs_path] = (st.st_size, st.st_mtime_ns, h.hexdigest())
        return h.hexdigest()

    def stat(self, abs_path, with_hash=True):
        st = os.stat(abs_path)
        is_dir = os.path.isdir(abs_path)
        top = abs_path == self.root
        out = {"path": "" if top else self.relpath(abs_path),
               "name": "" if top else os.path.basename(abs_path),
               "type": "dir" if is_dir else "file",
               "size": None if is_dir else st.st_size,
               "mtime": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(st.st_mtime)),
               "mtime_epoch": st.st_mtime}
        if with_hash and not is_dir:
            out["sha256"] = self.sha256(abs_path)
        return out

    def listdir(self, abs_path, with_hash=False):
        if not os.path.isdir(abs_path):
            raise FileStoreError(404, "not_found", "no such directory")
        return [self.stat(os.path.join(abs_path, n), with_hash) for n in sorted(os.listdir(abs_path))
                if not n.startswith(".")]

    def search(self, pattern="*", recursive=True, with_hash=False):
        out = []
        for d, dirs, files in os.walk(self.root):
            dirs[:] = sorted(x for x in dirs if not x.startswith("."))
            out += [self.stat(os.path.join(d, n), with_hash) for n in sorted(files)
                    if not n.startswith(".") and fnmatch.fnmatch(n, pattern)]
            if not recursive:
                break
        return out

    def usage_bytes(self):
        total = 0
        for d, _dirs, files in os.walk(self.root):
            for n in files:
                try:
                    total += os.path.getsize(os.path.join(d, n))
                except OSError:
                    pass
        return total

    # ---- changes ----

    def mkdir(self, abs_path):
        if abs_path == self.root:
            raise FileStoreError(400, "bad_path", "the root exists already")
        if os.path.isfile(abs_path):
            raise FileStoreError(409, "conflict", "a file with that name exists")
        os.makedirs(abs_path, exist_ok=True)
        return self.stat(abs_path)

    def write_stream(self, abs_path, stream, declared_length=None, overwrite=True):
        """Copy ``stream`` (has .read) to ``abs_path`` atomically, enforcing the
        per-file limit and the quota."""
        if abs_path == self.root or os.path.isdir(abs_path):
            raise FileStoreError(400, "bad_path", "the target is a directory")
        if os.path.exists(abs_path) and not overwrite:
            raise FileStoreError(409, "conflict", "the file exists (pass overwrite=1)")
        if declared_length is not None and declared_length > self.max_file_bytes:
            raise FileStoreError(413, "too_large", f"file exceeds the {self.max_file_bytes} byte limit")
        old = os.path.getsize(abs_path) if os.path.isfile(abs_path) else 0
        budget = self.quota_bytes - (self.usage_bytes() - old)
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".upload-", dir=os.path.dirname(abs_path))
        n, h = 0, hashlib.sha256()
        try:
            with os.fdopen(fd, "wb") as out:
                for chunk in iter(lambda: stream.read(2 ** 20), b""):
                    n += len(chunk)
                    if n > self.max_file_bytes:
                        raise FileStoreError(413, "too_large", f"file exceeds the {self.max_file_bytes} byte limit")
                    if n > budget:
                        raise FileStoreError(507, "quota_exceeded", f"the store quota of {self.quota_bytes} "
                                                                    "bytes is exceeded")
                    out.write(chunk)
                    h.update(chunk)
            umask = os.umask(0)            # mkstemp makes 0600; stored files are ordinary files
            os.umask(umask)
            os.chmod(tmp, 0o666 & ~umask)
            os.replace(tmp, abs_path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        st = os.stat(abs_path)
        self._sha[abs_path] = (st.st_size, st.st_mtime_ns, h.hexdigest())
        return self.stat(abs_path)

    def delete(self, abs_path, recursive=False):
        if abs_path == self.root:
            raise FileStoreError(400, "bad_path", "refusing to delete the root")
        if os.path.isdir(abs_path):
            if os.listdir(abs_path) and not recursive:
                raise FileStoreError(409, "not_empty", "the directory is not empty (pass recursive=1)")
            shutil.rmtree(abs_path)
        elif os.path.isfile(abs_path):
            os.unlink(abs_path)
            self._sha.pop(abs_path, None)
        else:
            raise FileStoreError(404, "not_found", "no such file")
