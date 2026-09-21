# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest

try:
    from ament_pep257.main import main
except ImportError:
    main = None


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    if main is None:
        pytest.skip("ament_pep257 is not installed")
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found code style errors / warnings"
