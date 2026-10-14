import json
from pathlib import Path
import unittest

import numpy as np

import ace_json


class AceReferenceTests(unittest.TestCase):
    def test_current_uav_comments_and_non_intrinsics_are_preserved(self):
        template = (Path(__file__).resolve().parents[1] / "reference_json" / "uav.json").read_text()
        before = json.loads(ace_json.strip_comments(template))
        camera = before["cameras"]["camera_0"]
        K = np.array(camera["camera_matrix"]["data"]).reshape(3, 3)
        K[0, 0] += 12.5
        P = np.column_stack((K, np.zeros(3)))
        for count in (5, 8):
            with self.subTest(distortion_count=count):
                D = np.arange(count) * 0.001
                output = ace_json.render(template, K, D, P, (1280, 960))
                after = json.loads(ace_json.strip_comments(output))
                self.assertEqual(list(after)[0], "schema_version")
                self.assertEqual(after["schema_version"], 1)
                for line in template.splitlines():
                    if "//" in line:
                        self.assertIn(line[line.index("//"):], output)
                allowed = {p[0] for p in ace_json.INTRINSIC_PATHS}
                for path in ace_json._leaf_diffs(before, after):
                    self.assertEqual(path[:2], ("cameras", "camera_0"))
                    self.assertIn(path[2], allowed)
                result = after["cameras"]["camera_0"]
                self.assertEqual(result["camera_matrix"]["data"], K.ravel().tolist())
                self.assertEqual(result["distortion_coefficients"]["cols"], count)
                self.assertEqual(result["distortion_coefficients"]["data"], D.tolist())

    def test_array_value_comments_do_not_become_calibration_numbers(self):
        template = '{"camera_matrix": {"data": [/* 3x3 */ 1, // fx 2026\n 2, 3]}}'
        output = ace_json._replace_array(template, "camera_matrix", [7, 8, 9], 5)
        self.assertIn("/* 3x3 */", output)
        self.assertIn("// fx 2026", output)
        self.assertEqual(json.loads(ace_json.strip_comments(output))["camera_matrix"]["data"],
                         [7, 8, 9])


if __name__ == "__main__":
    unittest.main()
