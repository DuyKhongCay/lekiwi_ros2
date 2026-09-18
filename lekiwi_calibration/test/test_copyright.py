# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest

try:
    from ament_copyright.main import main
except ImportError:
    main = None


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    if main is None:
        pytest.skip("ament_copyright is not installed")
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found errors in copyright"
