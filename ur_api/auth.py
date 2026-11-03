"""API access control: every request needs the token, except the few listed in OPEN.

Clients send "Authorization: Bearer <token>". The browser panel exchanges the token at POST /login for a cookie
(HttpOnly, SameSite=Strict), because <img src=/camera/stream> cannot send headers and because the strict cookie
stops another web page from firing GET /control/... at the arm through the operator's browser.
The token comes from env UR_API_TOKEN or <state dir>/api_token (created by ./run.sh, or here on first start).
/control/stop stays open on purpose: stopping must never fail on a lost login.
"""
import hashlib
import hmac
import logging
import os
import secrets

from flask import jsonify, make_response, request

log = logging.getLogger("ur_rest_api")
OPEN = {"/", "/ui", "/login", "/logout", "/control/stop"}
COOKIE = "ur_api_session"
_token = None


def enabled():
    """Env UR_API_AUTH=off turns the token check off: anyone who can reach the port can then move the arm."""
    return os.environ.get("UR_API_AUTH", "on").lower() not in ("off", "0", "false", "no")


def load_token(state_dir):
    """The API token: env UR_API_TOKEN, else the token file (created owner-only when missing)."""
    global _token
    if not enabled():
        log.warning("UR_API_AUTH=off: NO TOKEN. Anyone who can reach this port can move the arm")
        return
    t = os.environ.get("UR_API_TOKEN", "").strip()
    if not t:
        path = os.path.join(state_dir, "api_token")
        try:
            with open(path) as f:
                t = f.read().strip()
        except FileNotFoundError:
            t = ""
        if not t:
            t = secrets.token_urlsafe(32)
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w") as f:
                f.write(t + "\n")
            log.warning("created a new API token in %s", path)
    if len(t) < 16:
        raise SystemExit("the API token is shorter than 16 characters; set a longer UR_API_TOKEN")
    _token = t


def _session_value():
    return hashlib.sha256(("ur-api-session:" + _token).encode()).hexdigest()


def _authorized():
    h = request.headers.get("Authorization", "")
    if h.startswith("Bearer ") and hmac.compare_digest(h[7:].strip().encode(), _token.encode()):
        return True
    c = request.cookies.get(COOKIE, "")
    return hmac.compare_digest(c.encode(), _session_value().encode())


def gate():
    """before_request hook: None lets the request through."""
    if not enabled() or request.path in OPEN or _authorized():
        return None
    return jsonify({"error": "unauthorized", "msg": "send 'Authorization: Bearer <token>' (./run.sh token prints "
                    "it), or log in at /ui"}), 401


def allow_any_site(response):
    """after_request hook: with the token off, a website on another PC may call the API from its browser code too."""
    if not enabled():
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, PATCH, DELETE, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
    return response


def login():
    """Browser panel login: POST {"token": "..."} sets the session cookie."""
    if not enabled():
        return jsonify({"status": "ok", "msg": "no login needed: UR_API_AUTH=off"})
    d = request.get_json(silent=True)
    t = d.get("token") if isinstance(d, dict) else None
    if not isinstance(t, str) or not hmac.compare_digest(t.strip().encode(), _token.encode()):
        return jsonify({"error": "unauthorized", "msg": "wrong token"}), 401
    r = make_response(jsonify({"status": "ok"}))
    r.set_cookie(COOKIE, _session_value(), httponly=True, samesite="Strict", max_age=12 * 3600)
    return r


def logout():
    """Browser panel logout: clears the session cookie."""
    r = make_response(jsonify({"status": "ok"}))
    r.delete_cookie(COOKIE)
    return r
