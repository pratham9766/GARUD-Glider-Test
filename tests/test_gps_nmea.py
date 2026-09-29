import unittest
from unittest.mock import patch

import config
from sensors.gps import RealGPS


class FakeSerialPort:
    def __init__(self, sentences):
        self._sentences = iter(sentences)
        self.is_open = True

    def readline(self):
        try:
            return (next(self._sentences) + "\r\n").encode("ascii")
        except StopIteration:
            return b""

    def close(self):
        self.is_open = False


class SerialNMEAGPSTest(unittest.TestCase):
    def test_combines_gga_and_rmc_and_marks_stale_fix(self):
        port = FakeSerialPort([
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
            "$GPRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W",
        ])

        with patch("serial.Serial", return_value=port):
            gps = RealGPS()

        gga = gps.read()
        self.assertTrue(gga["fix_ok"])
        self.assertEqual(gga["fix_type"], "GPS")
        self.assertAlmostEqual(gga["latitude"], 48.1173, places=4)
        self.assertAlmostEqual(gga["longitude"], 11.5166667, places=4)
        self.assertAlmostEqual(gga["altitude"], 545.4)
        self.assertEqual(gga["satellites"], 8)
        self.assertAlmostEqual(gga["hdop"], 0.9)

        rmc = gps.read()
        self.assertTrue(rmc["fix_ok"])
        self.assertAlmostEqual(rmc["ground_speed_mps"], 22.4 * 0.514444)
        self.assertAlmostEqual(rmc["course_deg"], 84.4)

        gps._last_sentence_monotonic -= config.GPS_FIX_STALE_TIMEOUT_SEC + 1.0
        self.assertFalse(gps._serial_snapshot()["fix_ok"])

        gps.close()
        self.assertFalse(port.is_open)


if __name__ == "__main__":
    unittest.main()
