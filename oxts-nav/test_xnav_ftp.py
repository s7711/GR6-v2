import unittest

from xnav_ftp import RD_NAME, parse_listing

LISTING = [
    "Type=cdir;Modify=20091102000012;Perm=celmp;Unique=QgmbSvAh5T/Y/U4FbCVo18Lj/6g=; /",
    "-rw-r--r-- 1 owner group 32 Oct 9 1999 mobile.nsp",
    "-rw-r--r-- 1 owner group 1395 Oct 9 1999 mobile.cfg",
    "-rw-r--r-- 1 owner group 530167296 Oct 9 1999 260928_235913.rd",
    "drwxr-xr-x 1 owner group 0 Oct 9 1999 somedir",
]


class TestParseListing(unittest.TestCase):
    def test_names_and_sizes(self):
        self.assertEqual(
            parse_listing(LISTING),
            {"mobile.nsp": 32, "mobile.cfg": 1395, "260928_235913.rd": 530167296},
        )

    def test_rd_name(self):
        self.assertTrue(RD_NAME.match("260928_235913.rd"))
        self.assertFalse(RD_NAME.match("mobile.rd"))
        self.assertFalse(RD_NAME.match("../260928_235913.rd"))


if __name__ == "__main__":
    unittest.main()
