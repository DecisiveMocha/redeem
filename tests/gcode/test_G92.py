from MockPrinter import MockPrinter
import mock
from Path import Path

class G921_Tests(MockPrinter):

    @mock.patch("gcodes.G92.G92Path")
    def test_gcodes_G92_with_no_args(self, mock_G92Path):
        self.execute_gcode("G92")
        mock_G92Path.assert_called_with(dict.fromkeys(self.printer.AXES, 0), 0.05)

    @mock.patch("gcodes.G92.G92Path")
    def test_gcodes_G92_with_XYZ_zero(self, mock_G92Path):
        self.execute_gcode("G92 X0 Y0 Z0")
        mock_G92Path.assert_called_with(dict.fromkeys("XYZ", 0), 0.05)
