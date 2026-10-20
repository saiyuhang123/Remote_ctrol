import hashlib

from robot_patrol.map_metadata import expected_occupancy_sha256


def test_signed_ros_raster_hash_is_exact_and_does_not_iterate_cells_in_python():
    from array import array
    from robot_patrol.map_metadata import occupancy_sha256
    class RosRaster(array):
        def __iter__(self):
            raise AssertionError('large ROS map must use the buffer protocol')
    values=list(range(-128,128))
    expected=hashlib.sha256(bytes(v & 255 for v in values)).hexdigest()
    assert occupancy_sha256(RosRaster('b',values))==expected
    assert occupancy_sha256(values)==expected


# Golden rasters captured from ROS 2 Humble nav2_map_server for this 3x2 PGM.
# Source raster (top row first): [0, 128, 255, 64, 192, 254].
# OccupancyGrid data is stored bottom row first.
NAV2_GOLDEN_RASTERS = {
    ("trinary", 0): [100, 0, 0, 100, -1, 0],
    ("trinary", 1): [-1, 100, 100, 0, -1, 100],
    ("scale", 0): [100, 0, 0, 100, 62, 0],
    ("scale", 1): [0, 100, 100, 0, 63, 100],
    ("raw", 0): [64, -1, -1, 0, -1, -1],
    # nav2_map_server ignores negate in raw mode.
    ("raw", 1): [64, -1, -1, 0, -1, -1],
}


def test_expected_occupancy_matches_nav2_map_server_modes(tmp_path):
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n3 2\n255\n" + bytes([0, 128, 255, 64, 192, 254]))

    for (mode, negate), expected in NAV2_GOLDEN_RASTERS.items():
        yaml = tmp_path / f"{mode}_{negate}.yaml"
        yaml.write_text(
            f"image: {image.name}\nmode: {mode}\nresolution: 0.5\n"
            "origin: [1.0, 2.0, 0.0]\n"
            f"negate: {negate}\nfree_thresh: 0.25\noccupied_thresh: 0.65\n",
            encoding="utf-8",
        )
        width, height, digest = expected_occupancy_sha256(str(yaml))
        actual = hashlib.sha256(bytes(value & 0xFF for value in expected)).hexdigest()
        assert (width, height) == (3, 2)
        assert digest == actual, f"Nav2 raster mismatch for mode={mode} negate={negate}"


# SHA256 of the 256-cell OccupancyGrid emitted by real nav2_map_server for a
# 256x1 PGM containing every grayscale value 0..255. These goldens cover the
# threshold boundaries (including gray 204 with free_thresh=0.2), all map
# modes, negate on/off, and several threshold pairs.
NAV2_ALL_GRAY_DIGESTS = {
    ("trinary", 0, 0.10, 0.90): "5b554c9dd5144c1a24f73872c4b95aa1e34e6fdb73bf59f913b741ce35c76ced",
    ("trinary", 1, 0.10, 0.90): "1caff32d82c8f1194527ca78c0a780f48ff1fe29c51085a5d561d339e75f773b",
    ("trinary", 0, 0.20, 0.80): "6e7f2b34172498d8bc9af46f76b6f24fded9c7b571e8a67ec3a52aad4657e7e2",
    ("trinary", 1, 0.20, 0.80): "cfc20f1bf27ac3b0a2872783e9cadabd7d77e24d18a19843f5640ca55986eb47",
    ("trinary", 0, 0.25, 0.65): "af6784d72c3beb5c0c4a3cac604420d6938d1427ff3c74461ce786bfcc058056",
    ("trinary", 1, 0.25, 0.65): "ea9d9913bb290957a781d9c3522c8f7239120ad19863bef3fa8e49a087d09f01",
    ("trinary", 0, 0.30, 0.70): "f279c0962cfd7d0548d31af7d32001d645d59c7905152eb180758bdb277c3ddd",
    ("scale", 0, 0.10, 0.90): "cc85aa063dad6e5682ff71b78170c6f7e76ccd73858394cf811cfae59eb93892",
    ("scale", 1, 0.10, 0.90): "d3fd7ece1fad8a2dfe5181fc54b1c36cd633830232e3abdc823989c018bafce6",
    ("scale", 0, 0.20, 0.80): "6312dcea25954fcae7d621885341a37000904bca20206641960e816ec594db0f",
    ("scale", 1, 0.20, 0.80): "fd9f77cf72cc10ecbbcb9e101623fb2ad37e99a01231ec11ce08ab8e663752f1",
    ("scale", 0, 0.25, 0.65): "abe921519ae89dd37b74f5bb0468bb91c57f0169cd1e192533ac0c8a840e3593",
    ("scale", 1, 0.25, 0.65): "73df596a6b464a61ea957c2360d45f8aa9f01f785d5b7dad6b7ff5d3cfe46def",
    ("scale", 0, 0.30, 0.70): "2da0026a4cd283c5b61989d7111a655df42b79916a3fbf9fbe10e07348c06922",
    ("raw", 0, 0.20, 0.80): "cc60a51f1848b7ba312f77ab46a63b4f155dcf3df1128fafdfaae11a618c48f7",
    ("raw", 1, 0.20, 0.80): "cc60a51f1848b7ba312f77ab46a63b4f155dcf3df1128fafdfaae11a618c48f7",
    ("raw", 0, 0.25, 0.65): "cc60a51f1848b7ba312f77ab46a63b4f155dcf3df1128fafdfaae11a618c48f7",
    ("raw", 1, 0.25, 0.65): "cc60a51f1848b7ba312f77ab46a63b4f155dcf3df1128fafdfaae11a618c48f7",
}


def test_expected_occupancy_all_gray_matches_nav2(tmp_path):
    image = tmp_path / "all_gray.pgm"
    image.write_bytes(b"P5\n256 1\n255\n" + bytes(range(256)))

    for (mode, negate, free_thresh, occupied_thresh), expected_digest in \
            NAV2_ALL_GRAY_DIGESTS.items():
        yaml = tmp_path / f"{mode}_{negate}_{free_thresh}_{occupied_thresh}.yaml"
        yaml.write_text(
            f"image: {image.name}\nmode: {mode}\nresolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            f"negate: {negate}\nfree_thresh: {free_thresh}\n"
            f"occupied_thresh: {occupied_thresh}\n",
            encoding="utf-8",
        )
        width, height, digest = expected_occupancy_sha256(str(yaml))
        assert (width, height) == (256, 1)
        assert digest == expected_digest, (
            f"Nav2 all-gray raster mismatch for mode={mode} negate={negate} "
            f"free={free_thresh} occupied={occupied_thresh}")
