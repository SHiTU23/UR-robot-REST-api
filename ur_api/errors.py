"""The one client-error type of the API: every module raises ApiError and the
app turns it into {"error": code, "msg": msg, **extra} with ``status``."""


class ApiError(Exception):
    def __init__(self, status, code, msg, **extra):
        super().__init__(msg)
        self.status, self.code, self.msg, self.extra = status, code, msg, extra


def err(status, code, msg, path=None, **extra):
    if path is not None:
        extra["path"] = path
    return ApiError(status, code, msg, **extra)
