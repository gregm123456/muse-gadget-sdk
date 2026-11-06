# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class AtomS3RImuTest(unittest.TestCase):
    def test_sustained_shake_hangup(self):
        source = (ROOT / "main/atoms3r_imu.c").read_text()
        start = source.index("// Orientation and shake, derived from the samples.")
        end = source.index("// sensors.read result:", start)
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            (out / "atoms3r_gestures.inc").write_text(source[start:end])
            cmd = [
                *shlex.split(os.environ.get("CC", "cc")), "-std=c11",
                "-Wall", "-Wextra", "-Werror", "-I", str(out),
                str(ROOT / "tests/link_atoms3r_gestures_harness.c"),
                "-lm", "-o", str(out / "gestures"),
            ]
            compiled = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run([str(out / "gestures")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_command_is_advertised_and_dispatched_with_the_feature(self):
        noise = (ROOT / "main/noise_control.cpp").read_text()
        start = noise.index("#if CONFIG_HOMEHUB_ATOMS3R_IMU")
        self.assertIn('add_command(commands, "sensors.read"', noise[start:noise.index("#endif", start)])
        app = (ROOT / "main/app.c").read_text()
        start = app.index("static cJSON *on_ws_command(")
        dispatch = app[start:app.index("unsupported command", start)]
        block = dispatch[dispatch.index("#if CONFIG_HOMEHUB_ATOMS3R_IMU"):]
        self.assertIn('"sensors.read"', block[:block.index("#endif")])

    def test_source_is_built_only_with_the_feature(self):
        cmake = (ROOT / "main/CMakeLists.txt").read_text()
        match = re.search(r"if\(CONFIG_HOMEHUB_ATOMS3R_IMU\)(.*?)endif\(\)", cmake, re.S)
        self.assertIsNotNone(match)
        self.assertIn("atoms3r_imu.c", match.group(1))

    def test_config_blob_is_complete_and_licensed(self):
        header = (ROOT / "main/atoms3r_bmi270_config.h").read_text()
        self.assertIn("BSD-3-Clause", header)
        self.assertIn("Bosch Sensortec", header)
        body = header[header.index("bmi270_config_file[] = {"):]
        self.assertEqual(len(re.findall(r"0x[0-9a-fA-F]{2}", body)), 8192)


if __name__ == "__main__":
    unittest.main()
