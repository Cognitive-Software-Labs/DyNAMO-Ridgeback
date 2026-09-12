"""The field-oriented error shape shared by every configurator response.

It lives in its own module because job canonicalisation, run supervision, and
the results browser all raise it, and none of them may import each other.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ConfiguratorError(Exception):
    """One field-oriented error suitable for the browser response."""

    field: str
    code: str
    message: str
    suggested_profile: str | None = None

    def as_dict(self) -> dict[str, str]:
        result = {'field': self.field, 'code': self.code, 'message': self.message}
        if self.suggested_profile:
            result['suggested_profile'] = self.suggested_profile
        return result
