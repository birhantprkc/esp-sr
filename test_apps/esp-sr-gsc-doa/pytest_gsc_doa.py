import pytest
from pytest_embedded import Dut


@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    ['p4_gsc_doa'],
)
def test_gsc_doa_p4(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='gsc_doa', timeout=600)
