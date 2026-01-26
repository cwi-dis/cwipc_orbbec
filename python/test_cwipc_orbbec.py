import unittest
import cwipc
import _cwipc_orbbec
import os
import sys
import tempfile

if 0:
    # This code can be used to debug the C++ code in XCode:
    # - build for XCode with cmake
    # - build the cwipc_util project
    # - Fix the pathname for the dylib
    # - run `python3 test_cwipc_util`
    # - Attach to python in the XCode debugger
    # - press return to python3.
    #
    # A similar procedure works for debugging with Visual Studio under windows.
    import _cwipc_orbbec
    _cwipc_orbbec.cwipc_orbbec_dll_load('C:/Users/VRTogether/VRTogether/cwipc_orbbec/build/bin/RelWithDebInfo/cwipc_orbbec.dll')
    print('Type return after attaching in XCode debugger (pid=%d) - ' % os.getpid())
    sys.stdout.flush()
    sys.stdin.readline()

#
# Windows search path is horrible. Work around it for testing with an environment variable
#
if 'CWIPC_TEST_DLL' in os.environ:
    filename = os.environ['CWIPC_TEST_DLL']
    dllobj = _cwipc_orbbec.cwipc_orbbec_dll_load(filename)

#
# Find directories for test inputs and outputs
#
_thisdir=os.path.dirname(os.path.join(os.getcwd(), __file__))
_topdir=os.path.dirname(_thisdir)
TEST_FIXTURES_DIR=os.path.join(_topdir, "tests", "fixtures")
TEST_FIXTURES_PLAYBACK_CONFIG=os.path.join(TEST_FIXTURES_DIR, "input", "recording", "cameraconfig.json")
TEST_OUTPUT_DIR=os.path.join(TEST_FIXTURES_DIR, "output")
if not os.access(TEST_OUTPUT_DIR, os.W_OK):
    TEST_OUTPUT_DIR=tempfile.mkdtemp('cwipc_orbbec_test') # type: ignore

class TestApi(unittest.TestCase):
    
    def _open_grabber(self):
        try:
            grabber = _cwipc_orbbec.cwipc_orbbec("auto")
        except cwipc.CwipcError as arg:
            if str(arg) == 'cwipc_orbbec: no orbbec cameras found':
                self.skipTest(str(arg))
            raise
        return grabber
        
    def test_cwipc_orbbec(self):
        """Test that we can grab a orbbec image"""
        grabber = None
        pc = None
        try:
            grabber = self._open_grabber()
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            # It seems the first pointcloud could be empty. Unsure why...
            if pc.count() == 0:
                pc = grabber.get()
                self.assertIsNotNone(pc)
                assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def test_cwipc_orbbec_tileinfo(self):
        """Test that we can get tileinfo from a orbbec grabber"""
        grabber = None
        try:
            grabber = self._open_grabber()
            nTile = grabber.maxtile()
            self.assertGreaterEqual(nTile, 1)
            # Assure the non-tiled-tile exists and points nowhere.
            tileInfo = grabber.get_tileinfo_dict(0)
            self.assertIsNotNone(tileInfo)
            assert tileInfo # Only to keep linters happy
            self.assertIn('normal', tileInfo)
            self.assertIn('cameraName', tileInfo)
            self.assertIn('cameraMask', tileInfo)
            self.assertIn('ncamera', tileInfo)
            # Untrue if multiple realsenses connected: self.assertLessEqual(tileInfo['ncamera'], 1)
            # Test some minimal conditions for other tiles
            for i in range(1, nTile):
                tileInfo = grabber.get_tileinfo_dict(i)
                self.assertIsNotNone(tileInfo)
                assert tileInfo # Only to keep linters happy
                if i in (1, 2, 4, 8, 16, 32, 64, 128):
                    # These tiles should exist and have a normal and camera ID (which may be None)
                    self.assertIn('normal', tileInfo)
                    self.assertIn('cameraName', tileInfo)
        finally:
            if grabber: grabber.free()

    @unittest.skip("no fixtures yet")
    def test_cwipc_orbbecplayback(self):
        """Test that we can grab a orbbec image from the playback grabber"""
        grabber = None
        pc = None
        try:
            grabber = _cwipc_orbbec.cwipc_orbbecplayback(TEST_FIXTURES_PLAYBACK_CONFIG)
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()
            
    @unittest.skip("no fixtures yet")
    def test_cwipc_orbbecplayback_seek(self):
        """Test that we can grab a orbbec image from the playback grabber"""
        grabber = None
        pc = None
        try:
            grabber = _cwipc_orbbec.cwipc_orbbecplayback(TEST_FIXTURES_PLAYBACK_CONFIG)
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            result = grabber.seek(1600233)
            self.assertTrue(result)
            result = grabber.seek(5000000)
            self.assertFalse(result)
            pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def _verify_pointcloud(self, pc : cwipc.cwipc_wrapper) -> None:
        points = pc.get_points()
        self.assertGreater(len(points), 1)
        halfway = int((len(points)+1)/2)
        p0 = points[0].x, points[0].y, points[0].z, points[0].r, points[0].g, points[0].b
        p1 = points[halfway].x, points[halfway].y, points[halfway].z, points[halfway].r, points[halfway].g, points[halfway].b
        self.assertNotEqual(p0, p1)
   
if __name__ == '__main__':
    unittest.main()
