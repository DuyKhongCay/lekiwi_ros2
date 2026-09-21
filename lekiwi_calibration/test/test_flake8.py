# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest

try:
    from ament_flake8.main import main_with_errors
except ImportError:
    main_with_errors = None


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    if main_with_errors is None:
        pytest.skip("ament_flake8 is not installed")
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, "Found %d code style errors / warnings:\n" % len(
        errors
    ) + "\n".join(errors)
