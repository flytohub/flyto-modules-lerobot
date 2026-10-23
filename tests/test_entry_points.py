"""The three entry points a Flyto2 install finds this pack by, loaded by name.

Materialised with ``tuple()`` before indexing (Python 3.12 ``EntryPoints``).
Skipped when the package is not installed into this interpreter.
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, distribution, entry_points

import pytest

from flyto_modules_lerobot.capabilities import ADAPTER_ID, MODULE_PACK

GROUPS = (
    ("flyto.modules", MODULE_PACK),
    ("flyto2.external_adapters", ADAPTER_ID),
    ("flyto2.resource_discoverers", ADAPTER_ID),
)


@pytest.fixture(autouse=True)
def installed():
    try:
        distribution("flyto-modules-lerobot")
    except PackageNotFoundError:
        pytest.skip("flyto-modules-lerobot is not installed in this interpreter (pip install -e .)")


@pytest.mark.parametrize(("group", "name"), GROUPS)
def test_each_entry_point_loads_by_name(group, name):
    found = tuple(entry_points().select(group=group, name=name))
    assert len(found) == 1, (group, name)
    assert callable(found[0].load())
