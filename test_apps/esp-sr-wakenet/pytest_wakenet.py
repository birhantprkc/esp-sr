import pytest
from pytest_embedded import Dut


@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize('config', ['wn9_hilexin', 'wn10_hilexin'])
def test_wakenet(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='wn')


@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize('config', ['p4_wn9_hilexin', 'p4_wn10_hilexin'])
def test_wakenet_p4(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='wn')

